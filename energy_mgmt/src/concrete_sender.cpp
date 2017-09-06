#include "concrete_sender.h"

#include <adhoc_communication/EmAuction.h>
ros::Publisher auction_starting_pub, auction_reply_pub, auction_result_pub;
ros::ServiceClient fake_send_auction_sc;

ConcreteSender::ConcreteSender() {
    ros::NodeHandle nh;
//    send_auction_sc = nh.serviceClient<adhoc_communication::SendEmAuction>(my_prefix + "adhoc_communication/send_em_auction"); //TODO myprefix
    send_auction_sc = nh.serviceClient<adhoc_communication::SendEmAuction>("adhoc_communication/send_em_auction");
    
//    std::string auction_starting_topic = "adhoc_communication/send_em_auction/auction_starting"; 
//    std::string auction_reply_topic = "adhoc_communication/send_em_auction/auction_reply";
//    std::string auction_result_topic = "adhoc_communication/send_em_auction/auction_result"; 

//        auction_starting_pub = nh.advertise<adhoc_communication::EmAuction>(auction_starting_topic, 1000);
//        auction_reply_pub = nh.advertise<adhoc_communication::EmAuction>(auction_reply_topic, 1000);
//        auction_result_pub = nh.advertise<adhoc_communication::EmAuction>(auction_result_topic, 1000);

    fake_send_auction_sc = nh.serviceClient<adhoc_communication::SendEmAuction>("fake_network/fake_send_auction");
}

void ConcreteSender::sendNewAuction(bid_t bid, auction_t auction, std::string topic, unsigned int robot_id) {
    ROS_INFO("Sending bid for auction %u to other robots on topic %s", auction.auction_id, topic.c_str());
    adhoc_communication::SendEmAuction srv;
    srv.request.topic = topic;
    srv.request.dst_robot = "mc_robot_0";
    srv.request.source_robot = auction.auctioneer;
    srv.request.auction.auction = auction.auction_id;
    srv.request.auction.robot = auction.auctioneer;
    srv.request.auction.docking_station = auction.docking_station_id;
    srv.request.auction.bid.bid = bid.bid;
    srv.request.auction.bid.robot = bid.robot_id;
    srv.request.auction.starting_time = auction.starting_time;
    
//    ROS_DEBUG("Calling service: %s", send_auction_sc.getService().c_str());
//    while(!send_auction_sc.call(srv))
//        ROS_ERROR("Call to service %s failed: retyring...", send_auction_sc.getService().c_str());

    ROS_DEBUG("Calling service: %s", fake_send_auction_sc.getService().c_str());
    while(!fake_send_auction_sc.call(srv))
        ROS_ERROR("Call to service %s failed: retyring...", fake_send_auction_sc.getService().c_str());

    ROS_DEBUG("Call succeeded");
}

void ConcreteSender::sendBid(bid_t bid, auction_t auction, std::string topic, unsigned int robot_id) {
    ROS_INFO("Sending bid for auction %u to other robots on topic %s", auction.auction_id, topic.c_str());
    adhoc_communication::SendEmAuction srv;
    srv.request.topic = topic;
    srv.request.dst_robot = "robot_" + std::to_string(auction.auctioneer);
    srv.request.source_robot = bid.robot_id;
    srv.request.auction.auction = auction.auction_id;
    srv.request.auction.robot = auction.auctioneer;
    srv.request.auction.docking_station = auction.docking_station_id;
    srv.request.auction.bid.bid = bid.bid;
    srv.request.auction.bid.robot = bid.robot_id;
    srv.request.auction.starting_time = auction.starting_time;
    
//    ROS_DEBUG("Calling service: %s", send_auction_sc.getService().c_str());
//    while(!send_auction_sc.call(srv))
//        ROS_ERROR("Call to service %s failed: retyring...", send_auction_sc.getService().c_str());
        
    ROS_DEBUG("Calling service: %s", fake_send_auction_sc.getService().c_str());
    while(!fake_send_auction_sc.call(srv))
        ROS_ERROR("Call to service %s failed: retyring...", fake_send_auction_sc.getService().c_str());  
 
    ROS_DEBUG("Call succeeded");
}

void ConcreteSender::sendResults(bid_t bid, auction_t auction, std::string topic, unsigned int robot_id, std::vector<unsigned int> participants) {
    ROS_INFO("Sending result for auction %u to other robots on topic %s", auction.auction_id, topic.c_str());
    adhoc_communication::SendEmAuction srv;
    srv.request.topic = topic;
    srv.request.dst_robot = "mc_robot_0";
    srv.request.source_robot = auction.auctioneer;
    srv.request.auction.auction = auction.auction_id;
    srv.request.auction.robot = auction.auctioneer;
    srv.request.auction.docking_station = auction.docking_station_id;
    srv.request.auction.winning_robot = bid.robot_id;
    for(auto it = participants.begin(); it != participants.end(); it++)
        srv.request.auction.participants.push_back(*it);
        
//    ROS_DEBUG("Calling service: %s", send_auction_sc.getService().c_str());
//    while(!send_auction_sc.call(srv))
//        ROS_ERROR("Call to service %s failed: retyring...", send_auction_sc.getService().c_str());
        
    ROS_DEBUG("Calling service: %s", fake_send_auction_sc.getService().c_str());
    while(!fake_send_auction_sc.call(srv))
        ROS_ERROR("Call to service %s failed: retyring...", fake_send_auction_sc.getService().c_str()); 
        
    ROS_DEBUG("Call succeeded");
}
