void docking::wait_for_explorer() {
    std_msgs::Empty msg;
    ros::Duration(5).sleep();
    while(!explorer_ready) {
        ROS_INFO("Waiting for the explorer services to be ready...");
        pub_wait.publish(msg);
        ros::Duration(5).sleep();
        ros::spinOnce();
    }
    ros::Duration(5).sleep();
    
    ros::service::waitForService("explorer/distance");
    //sc_distance = nh.serviceClient<explorer::Distance>(my_prefix + "explorer/distance", true);
    sc_distance = nh.serviceClient<explorer::Distance>(my_prefix + "explorer/distance");
    for(int i = 0; i < 10 && !sc_distance; i++) {
        ROS_ERROR("No connection to service 'explorer/distance': retrying...");
        ros::Duration(3).sleep();
        //sc_distance = nh.serviceClient<explorer::Distance>(my_prefix + "explorer/distance", true);
    }
    ROS_INFO("Established persistent connection to service 'explorer/distance'");
    //ros::Duration(0.1).sleep();
    
    ros::service::waitForService("explorer/reachable_target");
    //sc_reachable_target = nh.serviceClient<explorer::DistanceFromRobot>(my_prefix + "explorer/reachable_target", true);
    sc_reachable_target = nh.serviceClient<explorer::DistanceFromRobot>(my_prefix + "explorer/reachable_target");
    for(int i = 0; i < 10 && !sc_reachable_target; i++) {
        ROS_ERROR("No connection to service 'explorer/reachable_target': retrying...");
        ros::Duration(3).sleep();
        //sc_reachable_target = nh.serviceClient<explorer::DistanceFromRobot>(my_prefix + "explorer/reachable_target", true);
    }
    ROS_INFO("Established persistent connection to service 'explorer/reachable_target'");
    //ros::Duration(0.1).sleep();
    
    ros::service::waitForService("explorer/robot_pose");
    //sc_robot_pose = nh.serviceClient<fake_network::RobotPosition>(my_prefix + "explorer/robot_pose", true);
    sc_robot_pose = nh.serviceClient<fake_network::RobotPositionSrv>(my_prefix + "explorer/robot_pose");      
    for(int i = 0; i < 10 && !sc_robot_pose; i++) {
        ROS_ERROR("No connection to service 'explorer/robot_pose': retrying...");
        ros::Duration(3).sleep();
        //sc_robot_pose = nh.serviceClient<fake_network::RobotPosition>(my_prefix + "explorer/robot_pose", true);   
    }
    ROS_INFO("Established persistent connection to service 'explorer/robot_pose'");    
    //ros::Duration(0.1).sleep();
    
//    ros::Duration(5).sleep();
    send_robot();
}


template <class T>
void establishPersistenServerConnection(ros::ServiceClient &sc, std::string service_name) {
/*
    ros::service::waitForService(service_name);
    for(int i = 0; i < 10 && !(*sc); i++) {
        ROS_FATAL("NO MORE CONNECTION!");
        ros::Duration(1).sleep();
        *sc = nh.serviceClient<T>(service_name, true);
    } 
    */
}


void docking::wait_for_explorer_callback(const std_msgs::Empty &msg) {
    ROS_INFO("Explorer is ready");
    explorer_ready = true;
}
