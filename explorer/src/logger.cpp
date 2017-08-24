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

    void map_info()
    {
        /*
        * Publish average speed of robot
        */
        ros::NodeHandle nh_pub_speed;
        ros::Publisher publisher_speed = nh_pub_speed.advertise<explorer::Speed>("avg_speed", 1);

        fs_csv.open(csv_file.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
        fs_csv << "#sim_time,wall_time,global_map_progress_percentage,exploration_travel_path_global_meters," //TODO(minor) maybe there is a better way to obtain exploration_travel_path_global_meters without modifying ExplorationPlanner...
                  "traveled_distance,"
                  "global_map_explored_cells,discovered_free_cells_count,"
                  "local_map_explored_cells,total_number_of_free_cells"
//                  "recharge_cycles,energy_consumption,frontier_selection_strategy"
               << std::endl;
        fs_csv.close();
        
        double last_moving_instant = 0;

        while (ros::ok() && !exploration_finished)
        {
            // double angle_robot = robotPose.getRotation().getAngle();
            // ROS_ERROR("angle of robot: %.2f\n", angle_robot);

            print_mutex_info("map_info()", "acquiring");
            costmap_mutex.lock();
            print_mutex_info("map_info()", "lock");

            double time = ros::Time::now().toSec() - time_start.toSec();

            map_progress.global_freespace = global_costmap_size();
            //map_progress.global_freespace = discovered_free_cells_count;
            map_progress.local_freespace = local_costmap_size();
            map_progress.time = time;
            
            if(robot_is_moving()) { //notice that since the loop is executed every N seconds, we won't have a very precise value, but given that we use approximation in computing the remaining battery life it is ok...
                double elapsed_time_in_moviment = ros::Time::now().toSec() - last_moving_instant;
                moving_time += elapsed_time_in_moviment;
            }
            last_moving_instant = ros::Time::now().toSec();
            
            map_progress_during_exploration.push_back(map_progress);
            if(free_cells_count <= 0 || discovered_free_cells_count <= 0)
                percentage = -1;
            else
                percentage = (float) (discovered_free_cells_count * 100) / free_cells_count; //this makes sense only if the environment has no cell that are free but unreachable (e.g.:if there is rectangle in the environment, if it's surface is not completely black, the cells inside its perimeters are considered as free cells but they are obviously unreachable...); to solve this problem we would need a smart way to exclude cells that are free but unreachable...

            //ROS_ERROR("%.0f", map_progress.global_freespace);
            //ROS_ERROR("%d", free_cells_count);
            //ROS_ERROR("%f", percentage);
            double exploration_travel_path_global =
                //F
                //(double)exploration->exploration_travel_path_global * costmap_resolution;
                exploration->exploration_travel_path_global_meters;

            if (battery_charge_temp >= battery_charge)
                energy_consumption += battery_charge_temp - battery_charge;
            battery_charge_temp = battery_charge;

            fs_csv.open(csv_file.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
            fs_csv << ros::Time::now().toSec() << "," << ros::WallTime::now().toSec() << "," 
                   << percentage << "," << exploration_travel_path_global << ","
                   << traveled_distance << ","
                   << map_progress.global_freespace << "," << discovered_free_cells_count << ","
                   << map_progress.local_freespace << "," << free_cells_count
//                   << battery_charge << "," << recharge_cycles << "," << energy_consumption << ","
                   << std::endl;
            fs_csv.close();

            costmap_mutex.unlock();
            print_mutex_info("map_info()", "unlock");

            ROS_DEBUG("Saving progress...");
            save_progress();
            ROS_DEBUG("Progress have been saved");

            // publish average speed
            explorer::Speed speed_msg;
            
//            speed_msg.avg_speed = exploration_travel_path_global / map_progress.time;
            speed_msg.avg_speed = exploration_travel_path_global / moving_time;

            publisher_speed.publish(speed_msg);

            ros::Duration(1.0).sleep();
        }
    }
    


    int global_costmap_size()
    {
        //occupancy_grid_global = costmap2d_global->getCostmap()->getCharMap();
        //int num_map_cells_ =
        //    costmap2d_global->getCostmap()->getSizeInCellsX() * costmap2d_global->getCostmap()->getSizeInCellsY();
        int free = 0;

        /*
        for (unsigned int i = 0; i < num_map_cells_; i++)
        {
            if ((int) occupancy_grid_global[i] == costmap_2d::FREE_SPACE)
            getCost(cell_x, cell_y)
            {
                free++;
            }
        }
        */
        
        //ROS_ERROR("%d", costmap2d_global->getCostmap()->getSizeInCellsX() * costmap2d_global->getCostmap()->getSizeInCellsY());
        
        for (unsigned int i = 0; i < costmap2d_global->getCostmap()->getSizeInCellsX(); i++)
            for (unsigned int j = 0; j < costmap2d_global->getCostmap()->getSizeInCellsY(); j++)
            {
                if (costmap2d_global->getCostmap()->getCost(i,j) == costmap_2d::FREE_SPACE)
                    free++;
            }
        
        //ROS_ERROR("%d", free);
        return free;
    }
    
    int total_size()
    {
        occupancy_grid_global = costmap2d_global->getCostmap()->getCharMap();
        return costmap2d_global->getCostmap()->getSizeInCellsX() * costmap2d_global->getCostmap()->getSizeInCellsY();
    }

    int local_costmap_size()
    {
        if (OPERATE_ON_GLOBAL_MAP == false)
        {
            occupancy_grid_local = costmap2d_local->getCostmap()->getCharMap();
            unsigned int num_map_cells_ =
                costmap2d_local->getCostmap()->getSizeInCellsX() * costmap2d_local->getCostmap()->getSizeInCellsY();
            int free = 0;

            for (unsigned int i = 0; i < num_map_cells_; i++)
            {
                if ((int)occupancy_grid_local[i] == costmap_2d::FREE_SPACE)
                {
                    free++;
                }
            }
            return free;
        }
        else
        {
            occupancy_grid_local = costmap2d_local_size->getCostmap()->getCharMap();
            unsigned int num_map_cells_ = costmap2d_local_size->getCostmap()->getSizeInCellsX() *
                                 costmap2d_local_size->getCostmap()->getSizeInCellsY();
            unsigned int free = 0;

            for (unsigned int i = 0; i < num_map_cells_; i++)
            {
                if ((int)occupancy_grid_local[i] == costmap_2d::FREE_SPACE)
                {
                    free++;
                }
            }
            return free;
        }
    }

    void log_stucked() {
    
        update_robot_state_2(stuck);
        this->indicateSimulationEnd();
        
        std::stringstream robot_number;
        robot_number << robot_id;

        std::string prefix = "/robot_";
        
        std::string status_directory = "/simulation_status_stuck";
        std::string robo_name = prefix.append(robot_number.str());
        std::string file_suffix(".stuck");

        std::string ros_package_path = ros::package::getPath("multi_robot_analyzer");
        std::string status_path = ros_package_path + status_directory;
        std::string status_file = status_path + robo_name + file_suffix;

        // TODO(minor): check whether directory exists
        boost::filesystem::path boost_status_path(status_path.c_str());
        if(!boost::filesystem::exists(boost_status_path))
            if(!boost::filesystem::create_directories(boost_status_path))
                ROS_ERROR("Cannot create directory %s.", status_path.c_str());
        std::ofstream outfile(status_file.c_str());
        outfile.close();
        ROS_INFO("Creating file %s to indicate end of exploration.",
        status_file.c_str());
        
        shutdown();
        
    }
    
    void log_stopped() {
    
        update_robot_state_2(stopped);
        
        std::stringstream robot_number;
        robot_number << robot_id;

        std::string prefix = "/robot_";
        
        std::string status_directory = "/simulation_status_stopped";
        std::string robo_name = prefix.append(robot_number.str());
        std::string file_suffix(".stuck");

        std::string ros_package_path = ros::package::getPath("multi_robot_analyzer");
        std::string status_path = ros_package_path + status_directory;
        std::string status_file = status_path + robo_name + file_suffix;

        // TODO(minor): check whether directory exists
        boost::filesystem::path boost_status_path(status_path.c_str());
        if(!boost::filesystem::exists(boost_status_path))
            if(!boost::filesystem::create_directories(boost_status_path))
                ROS_ERROR("Cannot create directory %s.", status_path.c_str());
        std::ofstream outfile(status_file.c_str());
        outfile.close();
        ROS_INFO("Creating file %s to indicate end of exploration.",
        status_file.c_str());
        
        this->indicateSimulationEnd();
        ros::Duration(10).sleep();
        
        shutdown();
        
    }

    void log_major_error(std::string text) {
        ROS_FATAL("%s", text.c_str());
        ROS_INFO("%s", text.c_str());
        
        major_errors++;
        
        major_errors_fstream.open(major_errors_file.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
        major_errors_fstream << "[MAJOR] " << robot_id << ": " << text << std::endl;
        major_errors_fstream.close();

        std::stringstream robot_number;
        std::stringstream error_counter;
        robot_number << robot_id;
        error_counter << major_errors;
        std::string prefix = "/robot_";
        
        std::string status_directory = "/simulation_status_error";
        std::string robo_name = prefix.append(robot_number.str());
        std::string file_name = robo_name.append(error_counter.str());
        std::string file_suffix(".major_error");

        std::string ros_package_path = ros::package::getPath("multi_robot_analyzer");
        std::string status_path = ros_package_path + status_directory;
        std::string status_file = status_path + file_name + file_suffix;

        // TODO(minor): check whether directory exists
        boost::filesystem::path boost_status_path(status_path.c_str());
        if(!boost::filesystem::exists(boost_status_path))
            if(!boost::filesystem::create_directories(boost_status_path))
                ROS_ERROR("Cannot create directory %s.", status_path.c_str());
        std::ofstream outfile(status_file.c_str());
        outfile.close();
        ROS_INFO("Creating file %s to indicate error",
        status_file.c_str());
        
    }
    
    void log_minor_error(std::string text) {
        ROS_FATAL("%s", text.c_str());
        ROS_INFO("%s", text.c_str());
        
        minor_errors++;
        
        major_errors_fstream.open(major_errors_file.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
        major_errors_fstream << "[minor] " << robot_id << ": " << text << std::endl;
        major_errors_fstream.close();

        std::stringstream robot_number;
        std::stringstream error_counter;
        robot_number << robot_id;
        error_counter << minor_errors;
        std::string prefix = "/robot_";
        
        std::string status_directory = "/simulation_status_minor_error";
        std::string robo_name = prefix.append(robot_number.str());
        std::string file_name = robo_name.append(error_counter.str());
        std::string file_suffix(".minor_error");

        std::string ros_package_path = ros::package::getPath("multi_robot_analyzer");
        std::string status_path = ros_package_path + status_directory;
        std::string status_file = status_path + file_name + file_suffix;

        // TODO(minor): check whether directory exists
        boost::filesystem::path boost_status_path(status_path.c_str());
        if(!boost::filesystem::exists(boost_status_path))
            if(!boost::filesystem::create_directories(boost_status_path))
                ROS_ERROR("Cannot create directory %s.", status_path.c_str());
        std::ofstream outfile(status_file.c_str());
        outfile.close();
        ROS_INFO("Creating file %s to indicate error",
        status_file.c_str());
        
    }
    
    void print_mutex_info(std::string function_name, std::string action) {
        log_mutex.lock();
        lock_file = log_path + std::string("lock.log");
        lock_fstream.open(lock_file.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
        lock_fstream << ros::Time::now() - time_start << ": " << function_name << ": " << action << std::endl;
        lock_fstream.close();
        log_mutex.unlock();
    }
    
    void store_travelled_distance() {
        ROS_INFO("Storing travelled distance");
        exploration->trajectory_plan_store(starting_x, starting_y);
        ROS_INFO("Stored");
        starting_x = pose_x;
        starting_y = pose_y;
    }
