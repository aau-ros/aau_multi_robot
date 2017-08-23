void save_progress(bool final = false)
    {
        ros::Duration ros_time = ros::Time::now() - time_start;

        double exploration_time = ros_time.toSec();
        int navigation_goals_required = counter;
        double exploration_travel_path = (double)exploration->exploration_travel_path_global_meters;
//        double size_global_map =
//            map_progress_during_exploration.at(map_progress_during_exploration.size() - 1).global_freespace;

        std::string tmp_log;
        if (!final)
        {
            tmp_log = log_file + std::string(".tmp");
            fs.open(tmp_log.c_str(), std::fstream::in | std::fstream::trunc | std::fstream::out);
        }
        else
        {
            tmp_log = log_file;
            fs.open(tmp_log.c_str(), std::fstream::in | std::fstream::trunc | std::fstream::out);
        }

        /*
         * WRITE LOG FILE
         * Write all the output to the log file
         */
        time_t raw_time;
        struct tm *timeinfo;
        time(&raw_time);
        timeinfo = localtime(&raw_time);

        fs << "[Exploration]" << std::endl;
        fs << "time_file_written    			= " << asctime(timeinfo);  // << std::endl;
        fs << "start_time           			= " << time_start << std::endl;
        fs << "end_time             			= " << ros::Time::now() << std::endl;
        fs << "exploration_time    	            = " << exploration_time << std::endl;
        fs << "required_goals                   = " << navigation_goals_required << std::endl;
        fs << "unreachable_goals                = " << exploration->unreachable_frontiers.size() << std::endl;
        fs << "travel_path_overall  	        = " << exploration_travel_path << std::endl;
        fs << "number_of_completed_auctions     = " << exploration->number_of_completed_auctions << std::endl;
        fs << "number_of_uncompleted_auctions   = " << exploration->number_of_uncompleted_auctions << std::endl;
        fs << "frontier_selection_strategy      = " << frontier_selection << std::endl;
        fs << "costmap_size                     = " << costmap_width << std::endl;
        fs << "global costmap iterations        = " << global_costmap_iteration << std::endl;
        fs << "number of recharges              = " << recharge_cycles << std::endl;
        fs << "energy_consumption               = " << energy_consumption << std::endl;
        fs << "maximum_available_distance       = " << conservative_maximum_available_distance << std::endl;
        fs << "w1                               = " << w1 << std::endl;
        fs << "w2                               = " << w2 << std::endl;
        fs << "w3                               = " << w3 << std::endl;
        fs << "w4                               = " << w4 << std::endl;
        fs << "queue_distance                   = " << queue_distance << std::endl;
        fs << "auction_timeout                  = " << auction_timeout << std::endl;
        fs << "checking_vacancy_timeout         = " << checking_vacancy_timeout << std::endl;

        double param_double;
        int param_int;
        std::string param;
        /*param = robot_prefix + "/explorer/local_costmap/height";
            ros::param::get(param,param_double);*/
        nh.param<int>("local_costmap/height", param_int, -1);
        fs << "explorer_local_costmap_height 		= " << param_int << std::endl;

        /*param = robot_prefix + "/explorer/local_costmap/width";
            ros::param::get(param,param_double);*/
        nh.param<int>("local_costmap/width", param_int, -1);
        fs << "explorer_local_costmap_width 		= " << param_int << std::endl;

        param = robot_prefix + "/move_base/local_costmap/height";
        ros::param::get(param, param_double);
        fs << "move_base_local_costmap_height 		= " << param_double << std::endl;

        param = robot_prefix + "/move_base/local_costmap/width";
        ros::param::get(param, param_double);
        fs << "move_base_local_costmap_width 		= " << param_double << std::endl;

        param = robot_prefix + "/move_base/global_costmap/obstacle_layer/raytrace_range";
        ros::param::get(param, param_double);
        fs << "move_base_raytrace_range 		= " << param_double << std::endl;

        param = robot_prefix + "/move_base/global_costmap/obstacle_layer/obstacle_range";
        ros::param::get(param, param_double);
        fs << "move_base_obstacle_range 		= " << param_double << std::endl;

        //	    param = robot_prefix +
        //"/navigation/global_costmap/obstacle_layer/raytrace_range";
        nh.getParam("/global_costmap/obstacle_layer/raytrace_range", param_double);
        ros::param::get(param, param_double);
        fs << "explorer_raytrace_range 		= " << param_double << std::endl;

        // param = robot_prefix +
        // "/navigation/global_costmap/obstacle_layer/obstacle_range";
        //    ros::param::get(param,param_double);
        nh.getParam("/global_costmap/obstacle_layer/obstacle_range", param_double);
        fs << "explorer_obstacle_range 		= " << param_double << std::endl;

        if (final)
            fs << "complete             			= "
               << "1" << std::endl;
        else
            fs << "complete             			= "
               << "0" << std::endl;

        fs.close();
        //            ROS_INFO("Wrote file %s\n", log_file.c_str());

        /*
         * Inform map_merger to save maps
         */

        if (final)
        {
            map_merger::LogMaps log;
            log.request.log = 3;  /// request local and global map
            ROS_INFO("Logging");
            if (!mm_log_client.call(log))
                ROS_ERROR("Could not call map_merger service to store log.");
        }
    }

void initLogPath()
    {
        /*
         * CREATE LOG PATH
         * Following code enables to write the output to a file
         * which is localized at the log_path
         */

        nh.param<std::string>("log_path", log_path, "");

        original_log_path = log_path;

        std::stringstream robot_number;
        robot_number << robot_id;
        std::string prefix = "/robot_";
        std::string robo_name = prefix.append(robot_number.str());

        log_path = log_path.append("/explorer");
        log_path = log_path.append(robo_name);
        ROS_INFO("Logging files to %s", log_path.c_str());

        boost::filesystem::path boost_log_path(log_path.c_str());
        if (!boost::filesystem::exists(boost_log_path))
            try
            {
                if (!boost::filesystem::create_directories(boost_log_path))
                    ROS_ERROR("Cannot create directory %s.", log_path.c_str());
            }
            catch (const boost::filesystem::filesystem_error &e)
            {
                ROS_ERROR("Cannot create path %s.", log_path.c_str());
            }

        log_path = log_path.append("/");
    }


void indicateSimulationEnd()
    {
        /// FIXME: remove this stuff once ported to multicast
        
        std::stringstream robot_number;
        robot_number << robot_id;

        std::string prefix = "/robot_";
        
        std::string status_directory = "/simulation_status";
        std::string robo_name = prefix.append(robot_number.str());
        std::string file_suffix(".finished");

        std::string ros_package_path = ros::package::getPath("multi_robot_analyzer");
        std::string status_path = ros_package_path + status_directory;
        std::string status_file = status_path + robo_name + file_suffix;

        // TODO(minor): check whether directory exists
        boost::filesystem::path boost_status_path(status_path.c_str());
        if(!boost::filesystem::exists(boost_status_path))
            if(!boost::filesystem::create_directories(boost_status_path))
                ROS_ERROR("Cannot create directory %s.", status_path.c_str());
        
        ROS_INFO("Creating file %s to indicate end of exploration.", status_file.c_str());
        std::ofstream outfile(status_file.c_str());
        outfile.close();
        if(!boost::filesystem::exists(status_file)) {
            log_major_error("cannot create status file!!!");
            //exit(-1);   
        } else
            ROS_INFO("Status file created successfully");
        
        if(percentage < 90 && robot_state != stuck) {
            log_major_error("low percentage (<90%)!!!");
        }
        
        if(percentage < 95 && robot_state != stuck) {
            log_minor_error("percentage < 95%");
        }
        
        ros::Duration(10).sleep();
        
        adhoc_communication::EmRobot msg;
        msg.id = robot_id;
        pub_finished_exploration_id.publish(msg);
        
        std_msgs::Empty msg2;
        pub_finished_exploration.publish(msg2);
        
        exploration_finished = true;
        
    }


void exploration_has_finished()
    {
        ros::Duration ros_time = ros::Time::now() - time_start;

        double exploration_time = ros_time.toSec();
        int navigation_goals_required = counter;
        double exploration_travel_path = (double)exploration->exploration_travel_path_global * 0.02;
        double size_global_map =
            map_progress_during_exploration.at(map_progress_during_exploration.size() - 1).global_freespace;
        ROS_INFO("overall freespace in the global map: %f", size_global_map);

        for (unsigned int i = 0; i < map_progress_during_exploration.size(); i++)
        {
            ROS_INFO("map progress: %f",
                     (map_progress_during_exploration.at(i).global_freespace / size_global_map) * 100);
        }

        ROS_DEBUG("******************************************");
        ROS_DEBUG("******************************************");
        ROS_DEBUG("TIME: %f sec  GOALS: %d  PATH: %f  COMPLETED AUCTIONS: %d  "
                  "UNCOMPLETED AUCTIONS: %d",
                  exploration_time, navigation_goals_required, exploration_travel_path,
                  exploration->number_of_completed_auctions, exploration->number_of_uncompleted_auctions);
        ROS_DEBUG("******************************************");
        ROS_DEBUG("******************************************");

        /*
         * WRITE LOG FILE
         * Write all the output to the log file
         */
        save_progress(true);
        ROS_INFO("Wrote file %s\n", log_file.c_str());

        /*
         * Inform map_merger to save maps
         */

        map_merger::LogMaps log;
        log.request.log = 3;  /// request local and global map
        ROS_INFO("Logging");
        if (!mm_log_client.call(log))
            ROS_ERROR("Could not call map_merger service to store log.");

        //#ifdef PROFILE
        // HeapProfilerStop();
        // ProfilerStop();
        //#endif
    }


    void log_map() {
        while (ros::ok() && !exploration_finished) {
            // call map_merger to log data
            map_merger::LogMaps log;
            log.request.log = 12;  /// request local and global map progress
            ROS_INFO("Calling map_merger service logOutput");
            if (!mm_log_client.call(log))
                ROS_ERROR("Could not call map_merger service to store log.");
            ROS_DEBUG("Finished service call.");
            ros::Duration(30).sleep();
        }
    }
