#include <ros/ros.h>
#include <ros/console.h>
#include "fake_network/SendMessage.h"
#include "fake_network/NetworkMessage.h"
#include "std_msgs/Empty.h"
#include <fake_network/RobotPosition.h>
//#include <adhoc_communication/EmRobot.h>

#define SSTR(x) static_cast<std::ostringstream &>((std::ostringstream() << std::dec << x)).str()

float wifi_range;
ros::Publisher pub;
std::vector<ros::ServiceServer> ss_send_message_list;
std::vector<ros::ServiceClient> sc_publish_message_list;
std::vector<ros::Publisher> pub_publish_message_list;
//std::vector<ros::ServiceClient> sc_robot_position_list;
std::vector<ros::Subscriber> sub_robot_position_list;
std::vector<ros::ServiceServer> ss_robot_position_list;
std::vector<ros::Subscriber> sub_finished_exploration_list;
struct robot_t {
    float x;
    float y;
    bool finished;
};
std::vector<robot_t> robot_list;
std::vector<bool> reachability_list;
int num_robots;
ros::Timer timer;

double euclidean_distance(double x1, double y1, double x2, double y2) {
    return sqrt( (x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) ) ; //TODO bad...
}

bool reachable(int src, int dst) {
    for(int j=0; j < num_robots; j++) {
        if(reachability_list[j] == true)
            continue;
        //ROS_ERROR("%f", euclidean_distance(robot_list[src].x, robot_list[src].y, robot_list[j].x, robot_list[j].y));
        if(wifi_range < 0 || euclidean_distance(robot_list[src].x, robot_list[src].y, robot_list[j].x, robot_list[j].y) <= wifi_range) {
            if(j == dst) {
                return true;
            }
            else {  
                reachability_list[j] = true;
                reachability_list[src] = true; //to avoid a "backtracking"
                reachable(j, dst);  
            }
        }
    }
    return false;
}

bool send_message(fake_network::SendMessage::Request  &req, fake_network::SendMessage::Response &res) {
    
    std::fill(reachability_list.begin(), reachability_list.end(), false);
    
    fake_network::NetworkMessage msg2;
    msg2.source_robot = req.source_robot;
    msg2.payload = req.payload;
    msg2.data_type = req.data_type;
    msg2.topic = req.topic;
    msg2.destination = req.destination;
    
    std::string source_robot_id_str = req.source_robot.substr(6,1); //TODO doesn't work if we have more than 10 robots, but a lot of already existing code doesn't work with more than 10 robots, so...
    int source_robot_id = atoi(source_robot_id_str.c_str());
    
    bool broadcast;
    int dest_robot_id;
    if(req.destination == "" || req.destination.substr(0,3) == "mc_")
        broadcast = true;
    else if(req.destination.substr(0,6) == "robot_") {
        broadcast = false;
        std::string dest = req.destination.substr(6,1); //TODO doesn't work if we have more than 10 robots, but a lot of already existing code doesn't work with more than 10 robots, so...
        dest_robot_id = atoi(dest.c_str());
        if(dest_robot_id >= num_robots)
            ROS_ERROR("dest_robot_id >= num_robots");
    }
    else
        ROS_ERROR("invalid destination!!!");
    
    for(int i=0; i < num_robots; i++) {
    
        if(source_robot_id == i)
            continue;
        
        if(robot_list[i].finished)
            continue;
        
        //if(req.topic == "map_other")
        //    ROS_ERROR("publishing map from robot %s to robot robot_%d", req.source_robot.c_str(), i);
    
        if(reachable(source_robot_id, i) && (broadcast || dest_robot_id == i) )
            pub_publish_message_list[i].publish(msg2);
        else
            ; //ROS_ERROR("robot_%d and robot_%d cannot communicate", source_robot_id, i);
    }
    
    return true;
}

void robot_absolute_position_callback(const fake_network::RobotPosition msg) {
    robot_list[msg.id].x = msg.world_x;
    robot_list[msg.id].y = msg.world_y;
    //ROS_ERROR("(%f, %f)", robot_list[msg.id].x, robot_list[msg.id].y);
}

void end_simulation(const ros::TimerEvent &event) {
    ros::shutdown();
}

//void finished_exploration_callback(const adhoc_communication::EmRobot msg) {
//    ROS_INFO("robot_%d has finished exploring", msg.id);
//    robot_list[msg.id].finished = true;
//    
//    bool all_finished = true;
//    for(int i=0; i<robot_list.size(); i++)
//        if(!robot_list[i].finished)
//            all_finished = false;
//            
//    if(all_finished) {
//        ros::NodeHandle nh;
//        timer = nh.createTimer(ros::Duration(180), end_simulation, true, true);
//    }
//}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "fake_network");
    ros::NodeHandle nh;
    
    nh.param<int>("fake_network/num_robots", num_robots, -1);
    nh.param<float>("fake_network/communication_range", wifi_range, -1);

    //pub = nh.advertise<std_msgs::Empty>("robot_1/test", 10);
    
    for(int i = 0; i < num_robots; i++) {
        std::string robot_prefix = "robot_" + SSTR(i) + "/";
        ss_send_message_list.push_back(nh.advertiseService(robot_prefix + "fake_network/send_message", send_message));   
        sc_publish_message_list.push_back(nh.serviceClient<fake_network::SendMessage>(robot_prefix + "adhoc_communication/publish_message"));
        //sc_publish_message_list.push_back(nh.serviceClient<fake_network::SendMessage>(robot_prefix + "test"));
        pub_publish_message_list.push_back(nh.advertise<fake_network::NetworkMessage>(robot_prefix + "adhoc_communication/publish_message_topic", 10000000));
//        sc_robot_position_list.push_back(nh.serviceClient<fake_network::RobotPosition>(robot_prefix + "explorer/robot_pose"));
        sub_robot_position_list.push_back(nh.subscribe(robot_prefix + "fake_network/robot_absolute_position", 10000000, robot_absolute_position_callback));
//        sub_finished_exploration_list.push_back(nh.subscribe(robot_prefix + "finished_exploration_id", 1000, finished_exploration_callback));
        //ss_robot_position_list.push_back(nh.advertiseService(robot_prefix + "fake_network/robot_absolute_position", robot_absolute_position_callback)); 
        robot_t robot;
        robot.x = 0;    //TODO not very good...
        robot.y = 0;
        robot.finished = false;
        robot_list.push_back(robot);
        reachability_list.push_back(false);
    }
    
//    fake_network::RobotPosition robot_position;
    
    // Frequency of loop
    double rate = 10; // Hz
    ros::Rate loop_rate(rate);
    while(ros::ok()){
     
        /*
        for(int i = 0; i < 3; i++) {
            if(sc_robot_position_list[i].call(robot_position)) {
                robot_list[i].x = robot_position.response.x;
                robot_list[i].y = robot_position.response.x;
                
            }
            ROS_ERROR("(%f, %f)", robot_list[i].x, robot_list[i].y);
        }
        */
    
        // get updates from subscriptions
        ros::spinOnce();
 
        // sleep for 1/rate seconds
        loop_rate.sleep();
        
        //ROS_ERROR("End of main loop");
    }
    

    return 0;
}
