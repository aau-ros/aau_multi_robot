/*
 * Packet.cpp
 *
 *  Created on: 21.08.2013
 *      Author: Günther Cwioro
 *
 *
 */

#include <string>
#include <list>

#include "Packet.h"
#include "functions.h"
#include "RoutedFrame.h"
#include "EthernetFrame.h"

Packet::Packet(RoutedFrame frame)
{
    this->id_ = frame.header_.packet_id;
    this->size_ = frame.header_.packet_size;
    this->data_type_ = frame.header_.payload_type;
    this->ts_ = getMillisecondsTime();
    this->frames_l_.clear();
    this->topic_ = frame.topic_;
    this->hostname_source_ = frame.hostname_source_;
    this->mc_group_ = frame.mc_g_name_;
    this->missed_sequences_l_.clear();
    highest_seq = 0;
    ts_last_frame_request = 0;
}

Packet::~Packet()
{
}

Packet::Packet()
{
}

unsigned long Packet::getSize()
{
    unsigned long size = 0;
    for (std::list<RoutedFrame>::iterator it = frames_l_.begin(); it != frames_l_.end(); ++it)
    {
        size += (*it).getSize();
    }

    size += sizeof (id_);
    size += sizeof (size_);
    size += sizeof (data_type_);
    size += sizeof (ts_);
    size += sizeof (topic_);
    size += sizeof (hostname_source_);
    size += sizeof (mc_group_);
    size += sizeof (uint32_t) * missed_sequences_l_.size();
    return size;
}

bool Packet::frameAlreadyExsits(RoutedFrame f)
{
    for (std::list<RoutedFrame>::iterator it = frames_l_.begin(); it != frames_l_.end(); ++it)
    {
        RoutedFrame & frame(*it);

        if (frame.header_.packet_sequence_num == f.header_.packet_sequence_num)
        {
            return true;
        }
    }
    return false;
}

void Packet::sortFrameList()
{
    if (this->wrong_sequence_)
    {
        /* Simple sort algo */
        list<RoutedFrame> ordered_frames;
        for (uint32_t i = 0; i < this->size_; i++)
        {
            bool found = false;
            for (std::list<RoutedFrame>::iterator it = frames_l_.begin();
                    it != frames_l_.end(); ++it)
            {
                RoutedFrame & frame(*it);
                bool doppelt = false;

                if (frame.header_.packet_sequence_num == i)
                {
                    if (doppelt)
                        ROS_ERROR("doppelt: %u", i);
                    else
                    {
                        ordered_frames.push_back(frame);
                        found = true;
                        doppelt = true;
                    }
                }
            }
            if (!found)
                ROS_ERROR("missing frame!!! %u", i);
        }
        this->frames_l_ = ordered_frames;
    }
    else
    {
        /* Fast sort algo */
        RoutedFrame first = RoutedFrame(frames_l_.front());

        while (first.header_.packet_sequence_num != 0)
        {
            frames_l_.pop_front();
            for (std::list<RoutedFrame>::iterator it = frames_l_.begin();
                    it != frames_l_.end(); ++it)
            {
                RoutedFrame & frame(*it);
                if (frame.header_.packet_sequence_num + 1
                    == first.header_.packet_sequence_num)
                {
                    frames_l_.insert(++it, first);
                    break;
                }
            }
            first = RoutedFrame(frames_l_.front());
        }
    }
}

void Packet::refreshLists()
{
    this->wrong_sequence_ = true;
    list<RoutedFrame> ordered_frames;
    /* Generates a new ordered frame list, in which every seq num is only included once*/
    for (uint32_t i = 0; i < this->size_; i++)
    {
        bool multiple_occurrences = false;
        for (std::list<RoutedFrame>::iterator it = frames_l_.begin();
                it != frames_l_.end(); ++it)
        {
            RoutedFrame & frame(*it);

            if (frame.header_.packet_sequence_num == i)
            {
                if (!multiple_occurrences)
                {
                    ordered_frames.push_back(frame);
                    multiple_occurrences = true; //"" Occurrences
                }
            }
        }
    }
    frames_l_ = ordered_frames;

    /* Generates a new missed sequences list */
    std::list<uint32_t> missed_sequences;
    for (uint32_t count = 0; count < this->size_; count++)
    {

        RoutedFrame f;
        f.header_.packet_sequence_num = count;

        if (!this->frameAlreadyExsits(f))
            missed_sequences.push_back(count);

    }
    if (missed_sequences.empty())
    {
        for (uint32_t count = 0; count < this->size_; count++)
        {

            RoutedFrame f;
            f.header_.packet_sequence_num = count;

            if (!this->frameAlreadyExsits(f))
                missed_sequences.push_back(count);
        }
    }
    this->missed_sequences_l_ = missed_sequences;

    //ROS_ERROR("list refreshed %u %u %u", this->size_, missed_sequences.size(),
    //	frames_l_.size());
}

inline bool Packet::isMcFrame()
{
    return mc_group_.compare("") != 0;
}

inline bool Packet::isNack()
{
    return isMcFrame() && size_ >= Packet::NACK_THRESHOLD;
}

bool Packet::addFrame(RoutedFrame f)
{
    /* Description:
     * Add the frame to the frame list of the packet, if the frame belongs to the packet.
     * This method is needed to build up the packet on the receiver side.
     * The method also checks, if frames are received in the right order.
     *
     *
     * Returns:
     * 		(true) if packet is complete and ready to publish
     *
     * 		(false) in every other case
     */

    this->ts_ = getMillisecondsTime();
    
    if(this->size_ == this->frames_l_.size())
        return true;

    if (this->highest_seq < f.header_.packet_sequence_num)
        this->highest_seq = f.header_.packet_sequence_num;

    if (this->isMcFrame() && this->frameAlreadyExsits(f) == true)
    {
        this->refreshLists();
        return false;
    }

    if (frames_l_.size() == 0 && f.header_.packet_sequence_num != 0)
    {
        for (uint32_t mis_seq = 0; mis_seq < f.header_.packet_sequence_num; mis_seq++)
            missed_sequences_l_.push_back(mis_seq);
    }

    if (f.header_.packet_id != this->id_)
    {
        ROS_ERROR("packet error");
        exit(22);
    }

    if ((unsigned) frames_l_.size() < size_)
    {
        bool missed_sequence_frame = false;

        /* check if frame is on with a missed sequence*/
        for (std::list<uint32_t>::iterator it = missed_sequences_l_.begin(); it != missed_sequences_l_.end(); ++it)
        {
            if (f.header_.packet_sequence_num == *it)
            {
                //ROS_ERROR("Frame was in missed squ %u",f.header_.packet_sequence_num);
                missed_sequence_frame = true;
                missed_sequences_l_.erase(it); //todo try remove
                break;
            }
        }

        if (!missed_sequence_frame)
        {
            //std::list<RoutedFrame>::reverse_iterator iter = frames_l_.rbegin(); //points to the last instered frame

            RoutedFrame & last_frame(*frames_l_.rbegin());

            /*Check if the new frame has the expected sequence*/
            if (last_frame.header_.packet_sequence_num == f.header_.packet_sequence_num - 1 || frames_l_.size() == 0)
            {
                frames_l_.push_back(f);
            }
            else if (last_frame.header_.packet_sequence_num < f.header_.packet_sequence_num)
            {

                /*Insert all sequence numbers between the current incoming frame and the last inserted one*/
                for (uint32_t i = 1; i < f.header_.packet_sequence_num - last_frame.header_.packet_sequence_num; i++)
                {
                    missed_sequences_l_.push_front(last_frame.header_.packet_sequence_num + i);

                }

                frames_l_.push_back(f);
            }
            else
            {

                //	this->refreshLists();
                //return this->addFrame(f);
            }
        }
            /*Missed sequence frame*/
        else
        {
            frames_l_.push_front(f);
        }

        if (frames_l_.size() == size_)
        {
            this->wrong_sequence_ = true;
            sortFrameList();

            return frames_l_.size() == size_;
        }
        else
            return false;
    }

    return false;
}

std::string Packet::getPayload()
{
    /* Description:
     * Build the full payload string from the single frames
     *
     * Returns:
     * 		"" if packet is incomplete
     *
     * 		[Full Payload] if packet is complete
     */

    //	this->refreshLists();
    std::string payload = "";
    if (frames_l_.size() >= size_)
    {

        this->sortFrameList();

        /* build payload from the sorted list*/
        uint32_t expected_sequence = 0;
        for (std::list<RoutedFrame>::iterator it = frames_l_.begin();
                it != frames_l_.end(); ++it)
        {
            RoutedFrame frame = *it;

            if (frame.header_.packet_sequence_num == expected_sequence++) //doublecheck if frame is in the right order
                payload.append(frame.payload_);
            else
            {
                ROS_ERROR("PACKET SEG FAULT! %u %u", frame.header_.packet_sequence_num ,expected_sequence );
                return std::string("");
            }
        }
    }

    return payload;
}

