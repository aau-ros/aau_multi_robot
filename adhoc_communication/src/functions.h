/*
 * functions.h
 *
 *  Created on: Apr 14, 2014
 *      Author: cwioro
 */

#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

struct mac {
    unsigned char mac_adr[6];

    mac()
    {
    }

    mac(unsigned char* m)
    {
        memcpy((void*) mac_adr, m, 6);
    }
};

template <class t>
void desializeObject(unsigned char* serialized_pose_stamp, uint32_t length, t* obj);
void initMacFromString(unsigned char* mac, const char* mac_str);
const char* getMacAsCStr(unsigned char* mac);
void sleepMS(int* milli_sec);
void sleepMS(int milli_sec);
std::string getBoolAsString(bool value);

const char* getPathAsCStr(std::list<mac> p);
bool isBufferInList(unsigned char* buffer, std::vector<std::string>* old_buffers, uint8_t last_inserted);
unsigned long getMillisecondsTime();

unsigned long getMillisecondsTime()
{
    struct timeval tv;
    if (gettimeofday(&tv, NULL) != 0) return 0;
    return (unsigned long) ((tv.tv_sec * 1000ul) + (tv.tv_usec / 1000ul));
}

bool isBufferInList(unsigned char* buffer, std::vector<std::string>* old_buffers, uint8_t last_inserted)
{
    uint8_t iterations = 0;
    while (iterations++ < old_buffers->size())
    {

        std::string t = old_buffers->at(last_inserted--);
        if (t.compare(std::string((const char*) buffer, t.size())) == 0)
            return true;


        if (last_inserted >= old_buffers->size())
            last_inserted = old_buffers->size() - 1;
    }

    return false;
}

std::string getPathAsStr(std::list<mac> p)
{
    /* Description:
     * returns a C-String that displayed the a path of macs in hex.
     */
    std::string path = "";
    mac m;
    for (std::list<mac>::iterator it = p.begin(); it != p.end(); ++it)
    {
        if (it != p.begin())
            path.append("|");

        m = *it;
        boost::format fmt("%02X-%02X-%02X-%02X-%02X-%02X");
        for (int i = 0; i != 6; ++i)
            fmt % static_cast<unsigned int> (m.mac_adr[i]);

        path.append(fmt.str());
    }

    return path;
}

bool compareMac(char* mac1, char* mac2)
{


    for (int i = 0; i < 6; i++)
    {
        if (mac1[i] != mac2[i])
        {

            return false;
        }
    }
    return true;
}

bool compareMac(const unsigned char mac1[6], const unsigned char mac2[6])
{
    //	return true;

    for (int i = 0; i < 6; i++)
    {
        if (mac1[i] != mac2[i])
        {

            return false;
        }
    }
    return true;

    return compareMac((char*) mac1, (char*) mac2);
}

std::list<uint32_t> getRandomNumbers(uint32_t return_list_size, uint32_t max)
{
    uint32_t max_numbers = max > return_list_size ? return_list_size : max;
    std::list<uint32_t> rand_n;
    while (rand_n.size() <= max_numbers)
    {
        uint32_t r = rand() % max + 1;
        bool number_exsists = false;
        for (std::list<uint32_t>::iterator it = rand_n.begin(); it != rand_n.end(); ++it)
        {
            if (*it == r)
                number_exsists = true;
        }
        if (!number_exsists)
            rand_n.push_front(r);
    }

    return rand_n;

}

template <class t>
std::string getSerializedMessage(t message)
{
    /* Description:
     * returns a serialized ROS message as string to send it over network.
     */
    uint32_t serial_size = ros::serialization::serializationLength(message);
    boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
    ros::serialization::OStream streamOut(buffer.get(), serial_size);
    ros::serialization::serialize(streamOut, message);

    std::string serializedMap = "";
    serializedMap.append((const char*) buffer.get(), serial_size);

    return serializedMap;
}


/*
const char* getMacAsCStr(unsigned char* mac)
{

    /* Description:
     * returns a C-String that displayed the mac in hex.
     */
/*
    boost::format fmt("%02X-%02X-%02X-%02X-%02X-%02X");
    for (int i = 0; i != 6; ++i)
        fmt % static_cast<unsigned int> (mac[i]);

    return fmt.str().c_str();
}*/

std::string getMacAsStr(unsigned char* mac)
{

    /* Description:
     * returns a C-String that displayed the mac in hex.
     */
    boost::format fmt("%02X-%02X-%02X-%02X-%02X-%02X");
    for (int i = 0; i != 6; ++i)
        fmt % static_cast<unsigned int> (mac[i]);

    return fmt.str();
}

void sleepMS(int* milli_sec)
{

    //boost::this_thread::sleep(boost::posix_time::milliseconds(*milli_sec));//todo
    //

    int sec = *milli_sec / 1000;
    int m_sec = *milli_sec % 1000; //*milli_sec - ( sec * 1000); 
    ros::Duration(sec, m_sec * 1000000).sleep();
}

void sleepMS(int milli_sec)
{
    sleepMS(&milli_sec);
}

std::string getBoolAsString(bool value)
{
    if (value)
        return std::string("true");
    else
        return std::string("false");
}

bool containsString(std::vector<std::string>* l ,std::string* s)
{
    for(std::vector<std::string>::iterator i = l->begin(); i != l->end() ; i++)
    {
        if((*i).compare(*s)==0)
            return true;
    }
    return false;
}

std::string getIntAsString(unsigned long number)
{
    std::stringstream ss; //create a stringstream
    ss << number; //add number to the stream
    return ss.str(); //return a string with the contents of the stream
}

void initMacFromString(unsigned char* mac, const char* mac_str)
{
    const char* hex_str = mac_str;

    std::string mac_as_string;
    unsigned int ch;

    for (; std::sscanf(hex_str, "%2x", &ch) == 1; hex_str += 3)
        mac_as_string += ch;

    memcpy(mac, mac_as_string.data(), 6);
}

#endif /* FUNCTIONS_H_ */
