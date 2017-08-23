int main(int argc, char **argv)
{
#ifdef PROFILE
    const char fname[3] = "TS";
    ProfilerStart(fname);
    HeapProfilerStart(fname);
#endif

    exploration_finished = false;

    /*
     * ROS::init() function needs argc and argv to perform
     * any argument and remapping that is provided by the
     * command line. The third argument is the name of the node
     */
    ros::init(argc, argv, "simple_navigation");

    /*
     * Create instance of Explorer
     */
    tf::TransformListener tf(ros::Duration(10));
    Explorer explorer(tf);

    /*
     * The ros::spin command is needed to wait for any call-back. This could for
     * example be a subscription on another topic. Do this to be able to receive a
     * message.
     */
    boost::thread thr_explore(boost::bind(&Explorer::explore, &explorer));

    /*
     * The following thread is only necessary to log simulation results.
     * Otherwise it produces unused output.
     */
    boost::thread thr_map(boost::bind(&Explorer::map_info, &explorer));
    
    boost::thread thr_log_map(boost::bind(&Explorer::log_map, &explorer));

    /* Create thread to periodically publish unexplored frontiers */
    boost::thread thr_frontiers(boost::bind(&Explorer::frontiers, &explorer));
    
    boost::thread thr_safety_checks(boost::bind(&Explorer::safety_checks, &explorer));
    
    boost::thread thr_update_distances(boost::bind(&Explorer::update_distances, &explorer));

    /*
     * FIXME
     * Which rate is required in order not to oversee
     * any callback data (frontiers, negotiation ...)
     */
    while (ros::ok())
    {
        if(!exploration_finished) { //TODO actually we should termine the thread when the exploration is over...
            //explorer.print_mutex_info("main()", "acquiring");
            //ROS_DEBUG("acquiring");
            //costmap_mutex.lock();  
            //explorer.print_mutex_info("main()", "lock");
            //ROS_DEBUG("lock");
            ros::spinOnce();
            //costmap_mutex.unlock();
            //explorer.print_mutex_info("main()", "unlock");
            //ROS_DEBUG("unlock");
        }
        ros::Duration(0.1).sleep();
    }

    /* Terminate threads */
    thr_explore.interrupt();
    thr_map.interrupt();
    thr_explore.join();
    thr_map.join();
    thr_log_map.interrupt();
    thr_log_map.join();
    thr_update_distances.interrupt();
    thr_update_distances.join();
    thr_safety_checks.interrupt();
    thr_safety_checks.join();
    thr_frontiers.interrupt();
    thr_frontiers.join();
    // TODO thr_frontiers...

#ifdef PROFILE
    HeapProfilerStop();
    ProfilerStop();
#endif

    return 0;
}
