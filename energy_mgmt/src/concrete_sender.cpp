#include "concrete_sender.h"

ConcreteSender::ConcreteSender() {
    ros::NodeHandle nh;
//    send_auction_sc = nh.serviceClient<adhoc_communication::SendEmAuction>(my_prefix + "adhoc_communication/send_em_auction"); //TODO myprefix
    send_auction_sc = nh.serviceClient<adhoc_communication::SendEmAuction>("adhoc_communication/send_em_auction");
}

void ConcreteSender::sendBid(bid_t bid, auction_t auction, std::string topic, unsigned int robot_id) {
    ROS_INFO("Sending bid for auction %u to other robots on topic %s", auction.auction_id, topic.c_str());
    adhoc_communication::SendEmAuction srv;
    srv.request.topic = topic;
//    srv.request.dst_robot = group_name;
//group_name = "mc_robot_0";
    srv.request.dst_robot = "mc_robot_0";
    srv.request.auction.auction = auction.auction_id;
    srv.request.auction.robot = auction.auctioneer;
    srv.request.auction.docking_station = auction.docking_station_id;
    srv.request.auction.bid.bid = bid.bid;
    srv.request.auction.bid.robot = bid.robot_id;
    srv.request.auction.starting_time = auction.starting_time;
    ROS_DEBUG("Calling service: %s", send_auction_sc.getService().c_str());
    while(!send_auction_sc.call(srv))
        ROS_ERROR("Call to service %s failed: retyring...", send_auction_sc.getService().c_str());
    ROS_DEBUG("Call succeeded");
}

void ConcreteSender::sendResults(bid_t bid, auction_t auction, std::string topic, unsigned int robot_id, std::vector<unsigned int> participants) {
    ROS_INFO("Sending result for auction %u to other robots on topic %s", auction.auction_id, topic.c_str());
    adhoc_communication::SendEmAuction srv;
    srv.request.topic = topic;
//    srv.request.dst_robot = group_name;
//group_name = "mc_robot_0";
    srv.request.dst_robot = "mc_robot_0";
    srv.request.auction.auction = auction.auction_id;
    srv.request.auction.robot = auction.auctioneer;
    srv.request.auction.docking_station = auction.docking_station_id;
//    srv.request.auction_result.winning_bid.bid = bid.bid;
    srv.request.auction.winning_robot = bid.robot_id;
    for(auto it = participants.begin(); it != participants.end(); it++)
        srv.request.auction.participants.push_back(*it);
    ROS_DEBUG("Calling service: %s", send_auction_sc.getService().c_str());
    while(!send_auction_sc.call(srv))
        ROS_ERROR("Call to service %s failed: retyring...", send_auction_sc.getService().c_str());
    ROS_DEBUG("Call succeeded");
}
