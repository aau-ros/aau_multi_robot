    std::string my_prefix = ""; //TODO
    sc_reachable_target = nh.serviceClient<explorer::DistanceFromRobot>(my_prefix + "explorer/reachable_target");

void DistanceComputer::checkReachability() {
    explorer::DistanceFromRobot srv_msg;
    srv_msg.request.x = (*it).x;
    srv_msg.request.y = (*it).y;
    
    //ros::service::waitForService("explorer/reachable_target");
    for(int j = 0; j < 10 && !sc_reachable_target; j++) {
        ROS_FATAL("NO MORE CONNECTION!");
        ros::Duration(1).sleep();
        //sc_reachable_target = nh.serviceClient<explorer::DistanceFromRobot>(my_prefix + "explorer/reachable_target", true);
    }

    if (sc_reachable_target.call(srv_msg))
        reachable = srv_msg.response.reachable;
    else
    {
        ROS_ERROR("Unable to check if %s is reachable, retrying later...", dsToStr(*it).c_str());
        continue;
    }
}
