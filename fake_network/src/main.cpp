#include <ros/ros.h>
#include <ros/console.h>
#include "fake_network/SendMessage.h"
#include "fake_network/NetworkMessage.h"
#include "std_msgs/Empty.h"

#define SSTR(x) static_cast<std::ostringstream &>((std::ostringstream() << std::dec << x)).str()

ros::Publisher pub;
std::vector<ros::ServiceServer> ss_send_message_list;
std::vector<ros::ServiceClient> sc_publish_message_list;
std::vector<ros::Publisher> pub_publish_message_list;
    

bool send_message(fake_network::SendMessage::Request  &req, fake_network::SendMessage::Response &res);

bool send_message(fake_network::SendMessage::Request  &req, fake_network::SendMessage::Response &res) {
    //ROS_ERROR("Called!");
    std_msgs::Empty msg;
    
    fake_network::NetworkMessage msg2;
    msg2.source_robot = req.source_robot;
    msg2.payload = req.payload;
    msg2.data_type = req.data_type;
    msg2.topic = req.topic;
    
    //pub.publish(msg);
    //ROS_ERROR("%s", req.topic.c_str());
    
    fake_network::SendMessage srv_msg;
    srv_msg.request.topic = "hello";
    
    std::string source_robot_id_str = req.source_robot.substr(6,1);
    int source_robot_id = atoi(source_robot_id_str.c_str());
    
    //ROS_ERROR("%s", req.source_robot.c_str());
    //ROS_ERROR("%s", source_robot_id_str.c_str());
    //ROS_ERROR("%d", source_robot_id);
    
    for(int i=0; i < sc_publish_message_list.size(); i++) {
        if(source_robot_id == i) {
            //ROS_ERROR("skip");
            continue;
        }    
        
            
        //ROS_ERROR("%s", sc_publish_message_list[i].getService().c_str());
        //sc_publish_message_list[i].call(srv_msg);
        pub_publish_message_list[i].publish(msg2);
        //ROS_ERROR("returned");
    }
    
    return true;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "fake_network");
    ros::NodeHandle nh;

    //pub = nh.advertise<std_msgs::Empty>("robot_1/test", 10);
    
    for(int i = 0; i < 3; i++) {
        std::string robot_prefix = "robot_" + SSTR(i) + "/";
        ss_send_message_list.push_back(nh.advertiseService(robot_prefix + "fake_network/send_message", send_message));   
        sc_publish_message_list.push_back(nh.serviceClient<fake_network::SendMessage>(robot_prefix + "adhoc_communication/publish_message"));
        //sc_publish_message_list.push_back(nh.serviceClient<fake_network::SendMessage>(robot_prefix + "test"));
        pub_publish_message_list.push_back(nh.advertise<fake_network::NetworkMessage>(robot_prefix + "adhoc_communication/publish_message_topic", 10));  
    }
    
    // Frequency of loop
    double rate = 0.5; // Hz
    ros::Rate loop_rate(rate);
    while(ros::ok()){
        // get updates from subscriptions
        ros::spinOnce();
 
        // sleep for 1/rate seconds
        loop_rate.sleep();
        
        //ROS_ERROR("End of main loop");
    }
    

    return 0;
}
