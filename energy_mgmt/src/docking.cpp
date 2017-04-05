#include <docking.h>

using namespace std;

docking::docking()
{
    /* Read parameters */
    nh.param("energy_mgmt/num_robots", num_robots, 1);
    nh.param("energy_mgmt/w1", w1, 0.25);
    nh.param("energy_mgmt/w2", w2, 0.25);
    nh.param("energy_mgmt/w3", w3, 0.25);
    nh.param("energy_mgmt/w4", w4, 0.25);
    nh.param("energy_mgmt/x", origin_absolute_x, 0.0);
    nh.param("energy_mgmt/y", origin_absolute_y, 0.0);
    nh.param("energy_mgmt/distance_close", distance_close, 8.0);
    nh.param<string>("energy_mgmt/move_base_frame", move_base_frame, "map");
    nh.param<string>("energy_mgmt/robot_prefix", robot_prefix, "");
    nh.param<std::string>("log_path", log_path, "");

    /* Initialize robot name */
    if (robot_prefix.empty())
    {
        /* Empty_prefix: hardware platform (i.e., real experiment) */
        char hostname[1024];
        hostname[1023] = '\0';
        gethostname(hostname, 1023);
        robot_name = string(hostname);
        robot_id = 0;
        exit(0);
        my_prefix = "robot_" + SSTR(robot_id) + "/"; //TODO
    }
    else
    {
        /* Prefix is set: simulation */
        robot_name = robot_prefix; //TODO we need this? and are we use taht it must be equal to robot_refix (there is an unwanted '/' maybe...)

        /* Read robot ID number: to do this, it is required that the robot_prefix is in the form "/robot_<id>", where
         * <id> is the ID of the robot */
        // TODO what if there are more than 10 robots and so we have robot_10: this line of code will fail!!!
        robot_id = atoi(robot_prefix.substr(7, 1).c_str());
        ROS_DEBUG("Robot prefix: %s; robot id: %d", robot_prefix.c_str(), robot_id);

        /* Since we are in simulation and we use launch files with the group tag, prefixes to topics are automatically
         * added: there is no need to manually specify robot_prefix in the topic name */
        // my_prefix = "docking/"; //TODO
        my_prefix = "";
        //my_node = "energy_mgmt/"
        my_node = "";
    }

    // initialize robot struct
    robot_t robot;
    robot.id = robot_id;
    robot.state = active;
    robots.push_back(robot);

    // initialize service clients
    // TODO names (robot_0 end dockign)
    //TODO save names in variables

    /* SERVICE CLIENTS */
    /* Adhoc communication services */
    sc_send_auction =
        nh.serviceClient<adhoc_communication::SendEmAuction>(my_prefix + "adhoc_communication/send_em_auction");
    sc_send_docking_station = nh.serviceClient<adhoc_communication::SendEmDockingStation>(
        my_prefix + "adhoc_communication/send_em_docking_station");
    sc_send_robot = nh.serviceClient<adhoc_communication::SendEmRobot>(my_prefix + "adhoc_communication/send_em_robot");
    
    /* General services */
    sc_trasform = nh.serviceClient<map_merger::TransformPoint>("map_merger/transformPoint");  // TODO
    sc_robot_pose = nh.serviceClient<explorer::RobotPosition>(my_prefix + "explorer/robot_pose");
    sc_distance_from_robot = nh.serviceClient<explorer::DistanceFromRobot>(my_prefix + "explorer/distance_from_robot");

    /* Subscribers */
    sub_battery = nh.subscribe(my_prefix + "battery_state", 100, &docking::cb_battery, this);
    sub_robots = nh.subscribe(my_prefix + "robots", 100, &docking::cb_robots, this);
    sub_jobs = nh.subscribe(my_prefix + "frontiers", 10000, &docking::cb_jobs, this);
    sub_robot = nh.subscribe(my_prefix + "explorer/robot", 100, &docking::cb_robot, this);
    sub_docking_stations = nh.subscribe(my_prefix + "adhoc_communication/send_em_docking_station", 100,
                                        &docking::cb_docking_stations, this);
                                        
    
    sub_auction_starting =
        nh.subscribe(my_prefix + "adhoc_communication/send_em_auction/new_auction", 100, &docking::cb_auction, this);
    sub_auction_reply =
        nh.subscribe(my_prefix + "adhoc_communication/send_em_auction/reply", 100, &docking::cb_auction_reply, this);
    sub_auction_winner_adhoc =
        nh.subscribe(my_prefix + "adhoc_communication/auction_winner", 100, &docking::cb_auction_result, this);
    sub_robot_position = nh.subscribe("goalPoint/goalPoint", 100, &docking::robot_position_callback, this);  // TODO
    sub_adhoc_new_best_ds = nh.subscribe("adhoc_new_best_docking_station_selected", 100, &docking::adhoc_ds, this);

    sub_charging_completed = nh.subscribe("charging_completed", 100, &docking::cb_charging_completed, this);

    sub_translate = nh.subscribe("translate", 100, &docking::cb_translate, this);
    sub_all_points = nh.subscribe("all_positions", 100, &docking::points, this);
    sub_check_vacancy = nh.subscribe("explorer/check_vacancy", 100, &docking::check_vacancy_callback, this);

    sub_check_vacancy =
        nh.subscribe("adhoc_communication/check_vacancy", 100, &docking::check_vacancy_callback, this);

    sub_ds_state_update =
        nh.subscribe("adhoc_communication/ds_state_update", 100, &docking::ds_state_update_callback, this);
    sub_robot_pose = nh.subscribe("amcl_pose", 100, &docking::robot_pose_callback, this);  // TODO not used

    /* Publishers */
    pub_ds = nh.advertise<std_msgs::Empty>("docking_station_detected", 100);
    pub_new_target_ds = nh.advertise<geometry_msgs::PointStamped>("new_target_docking_station_selected", 100);
    pub_adhoc_new_best_ds =
        nh.advertise<adhoc_communication::EmDockingStation>("adhoc_new_best_docking_station_selected", 100);
    pub_lost_own_auction = nh.advertise<std_msgs::Empty>("explorer/lost_own_auction", 100);
    pub_won_auction = nh.advertise<std_msgs::Empty>("explorer/won_auction", 100);
    pub_lost_other_robot_auction = nh.advertise<std_msgs::Empty>("explorer/lost_other_robot_auction", 100);
    
    pub_auction_result = nh.advertise<std_msgs::Empty>("explorer/auction_result", 100);
    
    pub_moving_along_path = nh.advertise<adhoc_communication::MmListOfPoints>("moving_along_path", 100);
    sub_moving_along_path = nh.subscribe("explorer/moving_along_path", 1, &docking::moving_along_path_callback, this);

    /* Timers */
    timer_finish_auction = nh.createTimer(ros::Duration(AUCTION_TIMEOUT), &docking::timerCallback, this, true, false);
    timer_restart_auction =
        nh.createTimer(ros::Duration(10), &docking::timer_callback_schedure_auction_restarting, this, true, false);

    /* Variable initializations */
    robot_state = fully_charged;
    charging_completed = false;
    going_charging_bool = false;
    auction_winner = false;
    lost_other_robot_auction = false;
    update_state_required = false;
    participating_to_auction = 0;
    managing_auction = false;
    going_to_ds = false;
    going_to_check_if_ds_is_free = false;
    need_to_charge = false;
    test = true;
    llh = 0;
    auction_id = 0;
    moving_along_path = false;

    /* Function calls */
    preload_docking_stations();
    initLogpath(); //TODO
    
    //TODO
    // force sending a broadcast message
    adhoc_communication::SendEmRobot srv_msg;
    srv_msg.request.topic = "robots"; //TODO should this service be called also somewhere else?
    srv_msg.request.robot.id = robot_id;
    srv_msg.request.robot.state = fully_charged;
    sc_send_robot.call(srv_msg);
    
    int graph[V][V] = {{0, 1, 0, 6, 0},
                       {1, 0, 1, 8, 5},
                       {0, 1, 0, 1, 7},
                       {6, 8, 1, 0, 1},
                       {0, 5, 7, 1, 0},
                      };
                     
    //compute_MST(graph);
}

void docking::robot_pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose)
{
    ROS_DEBUG("Received robot position");
    robot_x = pose->pose.pose.position.x;
    robot_y = pose->pose.pose.position.y;
}

void docking::preload_docking_stations()
{
    int index = 0;  // index of the DS: used to loop above all the docking stations inserted in the file
    double x, y;  // DS coordinates

    /* If the x-coordinate of DS with index <index> is found, it means that that DS is present in the file and must be
     * loaded */
    while (nh.hasParam("energy_mgmt/d" + SSTR(index) + "/x"))
    {
        /* Load coordinates */
        nh.param("energy_mgmt/d" + SSTR(index) + "/x", x, 0.0);
        nh.param("energy_mgmt/d" + SSTR(index) + "/y", y, 0.0);

        /* Store new DS */
        ds_t new_ds;
        new_ds.id = index;
        new_ds.vacant = true;
        translate_coordinates(x, y, &(new_ds.x), &(new_ds.y));
        undiscovered_ds.push_back(new_ds);

        /* Delete the loaded parameters (since they are not used anymore) and prepare to search for next DS */
        nh.deleteParam("energy_mgmt/d" + SSTR(index) + "/x");
        nh.deleteParam("energy_mgmt/d" + SSTR(index) + "/y");
        index++;
    }

    /* Print loaded DSs with their coordinates relative to the local reference system of the robot; only for debug
     * purposes */
    std::vector<ds_t>::iterator it;
    for (it = undiscovered_ds.begin(); it != undiscovered_ds.end(); it++)
        ROS_DEBUG("ds%d: (%f, %f)", (*it).id, (*it).x, (*it).y); 
    
}

void docking::compute_optimal_ds()
{

    /* Get current robot position */
    ros::service::waitForService("explorer/robot_pose");
    explorer::RobotPosition srv_msg;
    if (ros::service::exists("explorer/robot_pose", true))
        if (sc_robot_pose.call(srv_msg))
        {
            robot_x = srv_msg.response.x;
            robot_y = srv_msg.response.y;
            ROS_DEBUG("Robot position: (%f, %f)", robot_x, robot_y);
                     
        }
        else {
            ROS_ERROR("Call to service %s failed", sc_robot_pose.getService().c_str());
            return;  //TODO do better
        }
    else {
        ROS_ERROR("Service %s is not ready yet", sc_robot_pose.getService().c_str());
        return;    //TODO
   }

    /* Check if a new DS can be discovered */
    std::vector<ds_t>::iterator it;
    double dist;
    for(it = undiscovered_ds.begin(); it != undiscovered_ds.end(); it++) {
    
        /* Compute distance from robot */
        //dist = distance((*it).x, (*it).y);
        dist = ((*it).x - robot_x) * ((*it).x - robot_x) + ((*it).y - robot_y) * ((*it).y - robot_y); //TODO
        
        /* If the DS is inside a fiducial laser range, it can be considered discovered */
        if( dist > 0 && dist < LASER_RANGE ) {
        
            /* Store new DS in the vector of known DSs, and remove it from the vector of undiscovered DSs */
            ROS_DEBUG("Found new DS: ds%d", (*it).id); //TODO index make sense only in simulation
            undiscovered_ds.erase(it);
            ds.push_back(*it);
            ds_count++;
            
            /* Inform other robots about the new DS */
            //TODO call service to call cb_ds
            
            /* If it is the first DS that is encountered, set it as the currently optimal DS: this is just to initialize variable 'best_ds', so that the can compute the distance from the robot and the currently selected DS and compare it with the distance from the other DSs also when the node starts */
            if(ds_count == 1) {
                best_ds = ds.at(0);
                target_ds = best_ds;
                next_target_ds = target_ds;
            }
            
            /* Since an element from 'undiscovered-ds' was removed, we have to decrease 'it' by one to compensate the increment of the for loop, since, after the deletion of the element, all the elemtns are shifted by one position, and so we are already pointing to the next element, even without the +1 increment */
            it--;
            
        } else
            ROS_DEBUG("No new DS was encountered");
    }


    /* VERSION 1 */
    /*
    bool computed_new_optimal_ds = false;
    std::vector<ds_t>::iterator it = ds.begin();
    for (; it != ds.end(); it++)
        // if(optimal_ds_computed_once) {

        //TODO here I should consider that maybe there is a pending update of the DS...
        if ((best_ds.x - x) * (best_ds.x - x) + (best_ds.y - y) * (best_ds.y - y) >
            ((*it).x - x) * ((*it).x - x) + ((*it).y - y) * ((*it).y - y))
        {
            computed_new_optimal_ds = true;
            next_optimal_ds = *it;
        }
        else
            ;  // ROS_ERROR("\e[1;34m!!!!\e[0m");
    //} else {
    //    ROS_ERROR("\e[1;34mFirst computation of optimal DS: ds %d at (%f, %f)\e[0m", best_ds.id, best_ds.x,
    //    best_ds.y);
    //    optimal_ds_computed_once = true;
    //    best_ds = *it;
    //}

   if(computed_new_optimal_ds)
       if(participating_to_auction == 0 && !update_state_required) {
            ROS_ERROR("\e[1;34mNew optimal DS: ds%d (%f, %f)\e[0m", next_optimal_ds.id, next_optimal_ds.x,
   next_optimal_ds.y);
            best_ds = next_optimal_ds;
            geometry_msgs::PointStamped msg1;
            msg1.point.x = best_ds.x;
            msg1.point.y = best_ds.y;
            //pub_new_target_ds.publish(msg1);
        }
        else
            ROS_ERROR("\e[1;34mNext optimal DS: ds%d (%f, %f)\e[0m", next_optimal_ds.id, next_optimal_ds.x,
   next_optimal_ds.y);
     else
        ROS_DEBUG("Optimal DS unchanged");
    */

    /* VERSION 2 */  // TODO

    if (participating_to_auction == 0)
    {
        bool found_new_optimal_ds = false;
        if(DS_SELECTION_POLICY == 0) //TODO param, switch-case
        // closest
        {
            
            std::vector<ds_t>::iterator it = ds.begin();
            for (; it != ds.end(); it++)
                // if(optimal_ds_computed_once) {

                // TODO Disjkstra!!!
                if ((best_ds.x - robot_x) * (best_ds.x - robot_x) + (best_ds.y - robot_y) * (best_ds.y - robot_y) >
                    ((*it).x - robot_x) * ((*it).x - robot_x) + ((*it).y - robot_y) * ((*it).y - robot_y))
                {
                    found_new_optimal_ds = true;
                    best_ds = *it;
                }
        }
        else if(DS_SELECTION_POLICY == 1) // vacant
        {
            bool found_vacant_ds = false;
            std::vector<ds_t>::iterator it = ds.begin();
            for (; it != ds.end(); it++)
                if((*it).vacant) {
                    found_vacant_ds = true;
                    // TODO Disjkstra!!!
                    if ((best_ds.x - robot_x) * (best_ds.x - robot_x) + (best_ds.y - robot_y) * (best_ds.y - robot_y) >
                        ((*it).x - robot_x) * ((*it).x - robot_x) + ((*it).y - robot_y) * ((*it).y - robot_y))
                    {
                        found_new_optimal_ds = true;
                        best_ds = *it;
                    }
                }
            if(!found_vacant_ds)
                // closest //TODO use a function to call closest policy
                std::vector<ds_t>::iterator it = ds.begin();
                for (; it != ds.end(); it++)
                    // if(optimal_ds_computed_once) {

                    // TODO Disjkstra!!!
                    if ((best_ds.x - robot_x) * (best_ds.x - robot_x) + (best_ds.y - robot_y) * (best_ds.y - robot_y) >
                        ((*it).x - robot_x) * ((*it).x - robot_x) + ((*it).y - robot_y) * ((*it).y - robot_y))
                    {
                        found_new_optimal_ds = true;
                        best_ds = *it;
                    }
        }
        else if(DS_SELECTION_POLICY == 2) //opportune
        {
        /*
            int index_best_ds;
            for(int i=0, count_max=0; i < ds.size(); i++) {
                for(int j=0, count=0; j < jobs.size(); j++) {
                    int dist = distance(ds.at(i).x, ds.at(i).y, jobs.at(j).x, jobs.at(j).y);
                    if(dist < MAX_DISTANCE)
                        count++;
                 }
                if(count > count_max)    
            }
        */
            ds_t best_ds_opp;
            bool found_ds = true;
            int dist_min=100000; //TODO
            for(int i=0, count_max=0; i < ds.size(); i++) {
                for(int j=0, count=0; j < jobs.size(); j++) {
                    dist = distance(ds.at(i).x, ds.at(i).y, jobs.at(j).x, jobs.at(j).y);
                    if(dist < MAX_DISTANCE) {
                        found_ds = true;
                        if(distance(ds.at(i).x, ds.at(i).y) < dist_min) {
                            dist_min = distance(ds.at(i).x, ds.at(i).y);
                            best_ds = ds.at(i);
                            found_new_optimal_ds = true;            
                        }
                     }
                 }
            }
            if(!found_ds) {
                bool ds_found_with_mst=false;
            
                /*
                ; //spanning tree
                
                // construct ds graph
                int ds_graph[V][V];
                for(int i=0; i < ds.size(); i++)
                    for(int j=0; j<ds.size(); j++)
                        if(i == j)
                            ds_graph[i][j] = 0;
                        else {
                            int dist = distance(ds.at(i).x, ds.at(i).y, ds.at(j).x, ds.at(j).y);
                            if(dist < MAX_DISTANCE) {
                                ds_graph[i][j] = dist;
                                ds_graph[j][i] = dist;
                            }
                        }
                
                // construct MST starting from ds graph
                compute_MST(ds_graph);
                
                //TODO path to wich frontier???
                
                //TODO
                //find DS
                //find path
                //if found DS with spanning tree
                    //moving_along_path = true;
                
                
                
                
                */
                
                if(!moving_along_path) {
                    int first_step_x, first_step_y, second_step_x, second_step_y;
                    for(int i=0; i < ds.size(); i++) {
                        bool existing_eo;
                        for(int j=0; j < jobs.size(); j++)
                            if(distance(ds.at(i).x, ds.at(i).y, jobs.at(j).x, jobs.at(j).y) < MAX_DISTANCE) {
                                existing_eo = true;
                                for(int k=0; k < ds.size(); k++)
                                    if(k != i && distance(ds.at(i).x, ds.at(i).y, ds.at(k).x, ds.at(k).y) < MAX_DISTANCE )
                                        if( distance(ds.at(k).x, ds.at(k).y) < MAX_DISTANCE) {//TODO no MAX_DISTANCE, but available_distance
                                            moving_along_path = true;
                                            ds_found_with_mst = true;
                                            first_step_x = ds.at(k).x;
                                            first_step_y = ds.at(k).y;
                                            second_step_x = ds.at(i).x;
                                            second_step_y = ds.at(i).y;
                                        }
                            }
                    }
                }
                
                
                
                
                if(ds_found_with_mst) {
                    adhoc_communication::MmListOfPoints msg_path; //publish list of DS
                    msg_path.positions[0].x = 0;
                    msg_path.positions[0].y = 0;
                    msg_path.positions[1].x = 0;
                    msg_path.positions[1].y = 0;
                }else
                    ; //closest policy
            
            
            }

            

            
            
            
            
                            
            
        }
        else if(DS_SELECTION_POLICY == 3) //current
        {
            bool existing_eo = false;
            for(int i=0; i < jobs.size(); i++)
                //TODO use disjkstra and not use distance_close
                if((best_ds.x - jobs.at(i).x) * (best_ds.x - jobs.at(i).x) + (best_ds.y - jobs.at(i).y) * (best_ds.y - jobs.at(i).y) < distance_close) {
                    existing_eo = true;
                    break;
                }
            if(!existing_eo) {
                //closest //TODO use function
                std::vector<ds_t>::iterator it = ds.begin();
                for (; it != ds.end(); it++)
                    // if(optimal_ds_computed_once) {

                    // TODO Disjkstra!!!
                    if ((best_ds.x - robot_x) * (best_ds.x - robot_x) + (best_ds.y - robot_y) * (best_ds.y - robot_y) >
                        ((*it).x - robot_x) * ((*it).x - robot_x) + ((*it).y - robot_y) * ((*it).y - robot_y))
                    {
                        found_new_optimal_ds = true;
                        best_ds = *it;
                    }
            
            }
             
        }
        else if(DS_SELECTION_POLICY == 4) //flocking
        {
            for(int d=0, min_cost=numeric_limits<int>::max(); d < ds.size(); d++) {
                
    
                int count=0;
                for(int i=0; i < robots.size(); i++)
                    if(robots.at(i).target_ds == best_ds.id) //TODO best_ds or target_ds???
                        count++;
                double n_r = (double)count / num_robots;
                
                
                
                int sum_x = 0, sum_y = 0;
                for(int i=0; i < robots.size(); i++) {
                    sum_x += robots.at(i).x;
                    sum_y += robots.at(i).y;
                }
                double flock_x = sum_x / num_robots;
                double flock_y = sum_y / num_robots;
                double d_s = distance(ds.at(d).x, ds.at(d).y, flock_x, flock_y); //TODO normalization
                
                
                
                double swarm_direction_x = 0, swarm_direction_y = 0;
                for(int i=0; i < robots.size(); i++) {
                    double robot_i_target_ds_x, robot_i_target_ds_y;
                    
                    for(int k=0; k < ds.size(); k++)
                        if(robots.at(i).target_ds == ds.at(k).id) {
                            robot_i_target_ds_x = ds.at(k).x;
                            robot_i_target_ds_y = ds.at(k).y;
                        }
                            
                    
                    swarm_direction_x += robot_i_target_ds_x - robots.at(i).x;
                    swarm_direction_y += robot_i_target_ds_y - robots.at(i).y;
                }
                // tan ( param * PI / 180.0 );
                double alpha, rho, theta;
                
                /*
                if(swarm_direction_x == 0)
                    if(swarm_direction_y == 0)
                        ROS_ERROR("Degenerate case");
                    else if(swarm_direction_y > 0)
                        rho = 90;
                    else
                        rho = 270;
                else {
                    double atan_val = atan( abs(swarm_direction_y) / abs(swarm_direction_x) * 180.0 / PI);
                    if(swarm_direction_y >= 0 && swarm_direction_x > 0)
                        rho = atan;
                    else if(swarm_direction_y >= 0 && swarm_direction_x < 0)
                        rho = 360 - atan;
                    else if(swarm_direction_y <= 0 && swarm_direction_x > 0)
                        rho = 360 - atan;
                    else if(swarm_direction_y <= 0 && swarm_direction_x < 0)
                        rho = atan;
                }
                alpha = atan( abs(ds.at(d).y - robot_y) / abs(ds.at(d).x - robot_x) );
                */
                
                rho = atan2(swarm_direction_y, swarm_direction_x) * 180 / PI;
                alpha = atan2( (ds.at(d).y - robot_y) , (ds.at(d).x - robot_x) ) * 180 / PI;
                
                if(alpha > rho)
                    theta = alpha - rho;
                else
                    theta = rho - alpha;
                
                double theta_s = theta / 180;
                
                
                
                double d_f = 0.0; //TODO
                
                
                
                double cost = n_r + d_s + theta_s + d_f;
                if(cost < min_cost) {
                    found_new_optimal_ds = true;
                    min_cost = cost;
                    best_ds = ds.at(d);
                }   
            
            }  
            
        }
        
        //adhoc_communication::MmListOfPoints msg_points;
        //pub_moving_along_path.publish(msg_points);
        
        //ROS_ERROR("!!!!!!!!!!!!!!!%f", atan(-1) * 180.0 / PI );
        

        /* If a new optimal DS has been found, parameter l4 of the charging likelihood function must be updated */
        if (found_new_optimal_ds)
        {
            ROS_ERROR("New optimal DS: ds%d (%f, %f)", best_ds.id, best_ds.x, best_ds.y);
            geometry_msgs::PointStamped msg1;
            msg1.point.x = best_ds.x;
            msg1.point.y = best_ds.y;
            // pub_new_target_ds.publish(msg1); //TODO

            update_l4();
        }
        else
            ROS_DEBUG("Optimal DS unchanged");
    }
    else
        ROS_DEBUG("There are still some pending auctions: cannot update optimal DS");


     adhoc_communication::SendEmRobot robot_msg;
     robot_msg.request.topic = "adhoc_communication/send_em_robot";
     robot_msg.request.robot.id = robot_id;
     robot_msg.request.robot.x = robot_x;
     robot_msg.request.robot.y = robot_y;
     robot_msg.request.robot.selected_ds = best_ds.id;
     sc_send_robot.call(robot_msg);

    

}

void docking::points(const adhoc_communication::MmListOfPoints::ConstPtr &msg)  // TODO
{
    ;
}

void docking::adhoc_ds(const adhoc_communication::EmDockingStation::ConstPtr &msg)  // TODO
{
    ;  // ROS_ERROR("adhoc_ds!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
}

double docking::get_llh()
{
    /* The likelihood can be updated only if the robot is not participating to an auction */  // TODO explain better
    if (participating_to_auction == 0)
        llh = w1 * l1 + w2 * l2 + w3 * l3 + w4 * l4;
    return llh;
}

void docking::update_l1()  // TODO all update_l*
{
    // count vacant docking stations
    int num_ds_vacant = 0;
    for (int i = 0; i < ds.size(); ++i)
    {
        if (ds[i].vacant == true)
            ++num_ds_vacant;
    }

    // count active robots
    int num_robots_active = 0;
    for (int i = 0; i < robots.size(); ++i)
    {
        if (robots[i].state == active)
            ++num_robots_active;
    }

    // sanity checks
    if (num_ds_vacant < 0)
    {
        ROS_ERROR("Invalid number of vacant docking stations: %d!", num_ds_vacant);
        l1 = 0;
        return;
    }
    if (num_robots_active < 0)
    {
        ROS_ERROR("Invalid number of active robots: %d!", num_robots_active);
        l1 = 1;
        return;
    }

    // check boundaries
    if (num_ds_vacant > num_robots_active)
    {
        l1 = 1;
    }
    else if (num_robots_active == 0)
    {
        l1 = 0;
    }

    // compute l1
    else
    {
        l1 = num_ds_vacant / num_robots_active;
    }
}

void docking::update_l2()
{
    double time_run = battery.remaining_time_run;
    double time_charge = battery.remaining_time_charge;

    // sanity checks
    if (time_charge < 0)
    {
        ROS_ERROR("Invalid charging time: %.2f!", time_charge);
        l2 = 0;
        return;
    }
    if (time_run < 0)
    {
        ROS_ERROR("Invalid run time: %.2f!", time_run);
        l2 = 1;
        return;
    }
    if (time_run == 0 && time_charge == 0)
    {
        ROS_ERROR("Invalid run and charging times. Both are zero!");
        l2 = 1;
        return;
    }

    // compute l2
    l2 = time_charge / (time_charge + time_run);
}

void docking::update_l3()
{
    
    // count number of jobs: count frontiers and recheable frontiers
    int num_jobs = 0;
    int num_jobs_close = 0;
    for (int i = 0; i < jobs.size(); ++i)
    {
        ++num_jobs;
        if (distance(jobs[i].x, jobs[i].y, true) <= distance_close)  // use euclidean distance to make it faster //TODO
            ++num_jobs_close;
    }

    // sanity checks
    if (num_jobs < 0)
    {
        ROS_ERROR("Invalid number of jobs: %d", num_jobs);
        l3 = 1;
        return;
    }
    if (num_jobs_close < 0)
    {
        ROS_ERROR("Invalid number of jobs close by: %d", num_jobs_close);
        l3 = 1;
        return;
    }
    if (num_jobs_close > num_jobs)
    {
        ROS_ERROR("Number of jobs close by greater than total number of jobs: %d > %d", num_jobs_close, num_jobs);
        l3 = 0;
        return;
    }

    // check boundaries
    if (num_jobs == 0)
    {
        l3 = 1;
    }

    // compute l3
    else
    {
        l3 = (num_jobs - num_jobs_close) / num_jobs;
    }
    
}

void docking::update_l4()
{
    // get distance to docking station
    int dist_ds = -1;
    for (int i = 0; i < ds.size(); ++i)
    {
        if (ds[i].id == target_ds.id)
        {
            // dist_ds = distance(ds[i].x, ds[i].y);
            dist_ds = distance(ds[i].x, ds[i].y, true);  // TODO
            break;
        }
    }

    // get distance to closest job
    int dist_job = numeric_limits<int>::max();
    for (int i = 0; i < jobs.size(); ++i)
    {
        int dist_job_temp = distance(jobs[i].x, jobs[i].y, true);  // use euclidean distance to make it faster //TODO
        if (dist_job_temp < dist_job)
            dist_job = dist_job_temp;
    }

    // sanity checks
    if (dist_job < 0 || dist_job >= numeric_limits<int>::max())
    {
        ROS_ERROR("Invalid distance to closest job: %d", dist_job);
        l4 = 0;
        return;
    }
    if (dist_job == 0 && dist_ds == 0)
    {
        ROS_ERROR("Invalid distances to closest job and docking station. Both are zero!");
        l4 = 0;
        return;
    }

    // compute l4
    l4 = dist_job / (dist_job + dist_ds);
}

bool docking::auction(int docking_station, int id, int bid)  // TODO remove
{
    return true;
}

bool docking::auction_send_multicast(string multicast_group, adhoc_communication::EmAuction auction,
                                     string topic)  // TODO
{
    adhoc_communication::SendEmAuction auction_service;

    string destination_name = multicast_group + robot_name;

    ROS_INFO("Sending auction to multicast group %s on topic %s", destination_name.c_str(), topic.c_str());
    ROS_ERROR("Sending auction to multicast group %s on topic %s", destination_name.c_str(), topic.c_str());
    auction_service.request.dst_robot = destination_name;
    auction_service.request.auction = auction;
    auction_service.request.topic = topic;

    if (sc_send_auction.call(auction_service))
    {
        if (auction_service.response.status)
        {
            ROS_INFO("Auction was transmitted successfully.");
            return true;
        }
        else
        {
            ROS_WARN("Failed to send auction to multicast group %s!", destination_name.c_str());
            return false;
        }
    }
    else
    {
        ROS_WARN("Failed to call service %s/adhoc_communication/send_auction [%s]", robot_name.c_str(),
                 sc_send_auction.getService().c_str());
        return false;
    }
}

double docking::distance(double goal_x, double goal_y, bool euclidean)
{
    explorer::DistanceFromRobot srv_msg;
    srv_msg.request.x = goal_x;
    srv_msg.request.y = goal_y;

    //sc_distance_from_robot.call(srv_msg); //TODO

    // TODO
    /*
    tf::Stamped<tf::Pose> robotPose;
    if (!costmap->getRobotPose(robotPose))
    {
        ROS_ERROR("Failed to get RobotPose");
        return -1;
    }
    return distance(robotPose.getOrigin().getX(), robotPose.getOrigin().getY(), goal_x, goal_y, euclidean);
    */
    return distance(robot_x, robot_y, goal_x, goal_y, euclidean);
}

double docking::distance(double start_x, double start_y, double goal_x, double goal_y, bool euclidean)  // TODO
{
    double distance;

    // use euclidean distance
    if (euclidean)
    {
        double dx = goal_x - start_x;
        double dy = goal_y - start_y;
        distance = sqrt(dx * dx + dy * dy);
    }

    // calculate actual path length
    else
    {
        geometry_msgs::PoseStamped start, goal;

        start.header.frame_id = move_base_frame;
        start.pose.position.x = start_x;
        start.pose.position.y = start_y;
        start.pose.position.z = 0;

        goal.header.frame_id = move_base_frame;
        goal.pose.position.x = goal_x;
        goal.pose.position.y = goal_y;
        goal.pose.position.z = 0;

        vector<geometry_msgs::PoseStamped> plan;

        // if(nav.makePlan(start, goal, plan))
        //    distance =  plan.size() * costmap->getCostmap()->getResolution();
        // else
        distance = -1;
    }

    return distance;
}

void docking::robot_position_callback(const geometry_msgs::PointStamped::ConstPtr &msg)  // TODO
{
    ROS_DEBUG("Received robot position");
    robot_x = msg.get()->point.x;
    robot_y = msg.get()->point.y;
}

void docking::cb_battery(const energy_mgmt::battery_state::ConstPtr &msg)
{
    ROS_DEBUG("Received battery state");

    /* Store new battery state */
    battery.charging = msg.get()->charging;
    battery.soc = msg.get()->soc;
    battery.remaining_time_charge = msg.get()->remaining_time_charge;
    battery.remaining_time_run = msg.get()->remaining_time_run;
    battery.remaining_distance = msg.get()->remaining_distance;

    /* Update parameter l2 of charging likelihood function */
    update_l2();
}

void docking::cb_robot(const adhoc_communication::EmRobot::ConstPtr &msg)
{
    adhoc_communication::SendEmRobot srv_msg;
    srv_msg.request.topic = "robots"; //TODO should this service be called also somewhere else?
    srv_msg.request.robot.id = robot_id;
    srv_msg.request.robot.state = msg.get()->state;
    sc_send_robot.call(srv_msg);

    robot_state = static_cast<state_t>(msg.get()->state);

    // TODO correctly update the state!!!!
    if (msg.get()->state == exploring || msg.get()->state == fully_charged || msg.get()->state == moving_to_frontier)
        robots[robot_id].state = active;
    else
        robots[robot_id].state = idle;

    if (msg.get()->state == in_queue)
    {
        ROS_ERROR("\n\t\e[1;34mRobot in queue!!!\e[0m");

        /* Schedule next auction (a robot goes in queue only if it has lost an auction started by itself) */
        timer_restart_auction.setPeriod(ros::Duration(AUCTION_RESCHEDULING_TIME), true);
        timer_restart_auction.start();
        
        set_target_ds_vacant(true);


    }
    else if (msg.get()->state == going_charging)
    {
        ROS_ERROR("\n\t\e[1;34m Robo t going charging!!!\e[0m");
    }
    else if (msg.get()->state == charging)
    {
        ROS_ERROR("\n\t\e[1;34mRechargin!!!\e[0m");
        need_to_charge = false;  // TODO???

        set_target_ds_vacant(false);
    }
    else if (msg.get()->state == going_checking_vacancy)
    {
        ROS_ERROR("\n\t\e[1;34m going checking vacancy!!!\e[0m");
    }
    else if (msg.get()->state == checking_vacancy)
    {
        ROS_ERROR("\n\t\e[1;34m checking vacancy!!!\e[0m");
        adhoc_communication::SendEmDockingStation srv_msg;
        srv_msg.request.topic = "adhoc_communication/check_vacancy";
        srv_msg.request.docking_station.id = best_ds.id;
        sc_send_docking_station.call(srv_msg);
    } 
    else if (msg.get()->state == auctioning)
    {
        ROS_INFO("Robot need to recharge");
        need_to_charge = true;
        start_new_auction();
    }
    else if (msg.get()->state == fully_charged ||
             msg.get()->state == moving_to_frontier || msg.get()->state == exploring ||
             msg.get()->state == going_in_queue || msg.get()->state == finished)
    {
        ; // ROS_ERROR("\n\t\e[1;34midle!!!\e[0m");
    }
    else
    {
        ROS_ERROR("\n\t\e[1;34m none of the above!!!\e[0m");
        return;
    }

    robot_state = static_cast<state_t>(msg.get()->state);
}

// TODO
void docking::cb_robots(const adhoc_communication::EmRobot::ConstPtr &msg)
{

    // check if robot is in list already
    bool new_robot = true;
    for (int i = 0; i < robots.size(); ++i)
    {
        // robot is in list, update
        if (robots[i].id == msg.get()->id)
        {
            // update state //TODO
            if (msg.get()->state == exploring || msg.get()->state == fully_charged || msg.get()->state == moving_to_frontier)
                robots[i].state = active;
            else
                robots[i].state = idle;
            robots[i].x = msg.get()->x;
            robots[i].y = msg.get()->y;
            robots[i].target_ds = msg.get()->selected_ds;

            new_robot = false;
            break;
        }
    }

    // add new robot
    if (new_robot)
    {
        robot_t robot;
        robot.id = msg.get()->id;

         // update state //TODO
        if (msg.get()->state == exploring || msg.get()->state == fully_charged || msg.get()->state == moving_to_frontier)
            robot.state = active;
        else
            robot.state = idle;
        
        robot.x = msg.get()->x;
        robot.y = msg.get()->y;
        robot.target_ds = msg.get()->selected_ds;

        robots.push_back(robot);
        
        
        int count=0;
        for(int i=0; i < robots.size(); i++)
                count++;
                
        num_robots = count; //TODO also works for simulation? isn't this already known?
    }

    // update charging likelihood
    update_l1();
}

void docking::cb_jobs(const adhoc_communication::ExpFrontier::ConstPtr &msg)
{
    adhoc_communication::ExpFrontierElement frontier_element;
    
    jobs.clear();

    // add new jobs
    for (int i = 0; i < msg.get()->frontier_element.size(); ++i)
    {
        frontier_element = msg.get()->frontier_element.at(i);

        // check if it is a new job
        bool new_job = true;
        /*
        for (int j = 0; j < jobs.size(); ++j)
        {
            if (frontier_element.id == jobs.at(j).id)
            {
                new_job = false;
                break;
            }
        }
        */

        // store job if it is new
        if (new_job == true)
        {
            job_t job;
            job.id = frontier_element.id;
            job.x = frontier_element.x_coordinate;
            job.y = frontier_element.y_coordinate;
            jobs.push_back(job);
        }
    }

/*
    // remove completed jobs / frontiers
    for (int i = 0; i < jobs.size(); ++i)
    {
        // convert coordinates to cell indices
        unsigned int mx = 0;
        unsigned int my = 0;
        if (false)
        {
            // if(costmap->getCostmap()->worldToMap(jobs[i].x, jobs[i].y, mx, my) == false){
            ROS_ERROR("Could not convert job coordinates: %.2f, %.2f.", jobs[i].x, jobs[i].y);
            continue;
        }

        // get the 4-cell neighborhood with range 6
        vector<int> neighbors_x;
        vector<int> neighbors_y;
        for (int j = 0; j < 6; ++j)
        {
            neighbors_x.push_back(mx + i + 1);
            neighbors_y.push_back(my);
            neighbors_x.push_back(mx - i - 1);
            neighbors_y.push_back(my);
            neighbors_x.push_back(mx);
            neighbors_y.push_back(my + i + 1);
            neighbors_x.push_back(mx);
            neighbors_y.push_back(my - i - 1);
        }

        // check the cost of each neighbor
        bool unknown_found = false;
        bool obstacle_found = false;
        bool freespace_found = false;
        for (int j = 0; j < neighbors_x.size(); ++j)
        {
            int cost = 0;
            // unsigned char cost = costmap->getCostmap()->getCost(neighbors_x[j], neighbors_y[j]);
            if (cost == costmap_2d::NO_INFORMATION)
            {
                unknown_found = true;
            }
            else if (cost == costmap_2d::FREE_SPACE)
            {
                freespace_found = true;
            }
            else if (cost == costmap_2d::LETHAL_OBSTACLE)
            {
                obstacle_found = true;
            }
        }

        // delete frontier if either condition holds
        //  * no neighbor is unknown
        //  * at least one neighbor is an obstacle
        //  * no neighbor is free space
        if (unknown_found == false || obstacle_found == true || freespace_found == false)
        {
            jobs.erase(jobs.begin() + i);
            --i;
        }
    }

    // update charging likelihood
    update_l3();
    */
    update_l3();
}

void docking::cb_docking_stations(const adhoc_communication::EmDockingStation::ConstPtr &msg)
{

    // check if docking station is in list already
    bool new_ds = true;
    for (int i = 0; i < ds.size(); ++i)
    {
        // docking station is in list, update
        if (ds[i].id == msg.get()->id)
        {
            // coordinates don't match
            if (ds[i].x != msg.get()->x || ds[i].y != msg.get()->y)
                ROS_ERROR("Coordinates of docking station %d do not match: (%.2f,%.2f) != (%.2f,%.2f)", ds[i].id,
                          ds[i].x, ds[i].y, msg.get()->x, msg.get()->y);

            // update vacancy
            ds[i].vacant = msg.get()->vacant;

            new_ds = false;
            break;
        }
    }

    // add new docking station
    if (new_ds)
    {
        // ROS_ERROR("\e[1;34mNew docking station received\e[0m");
        ds_t s;
        s.id = msg.get()->id;
        s.x = msg.get()->x;
        s.y = msg.get()->y;
        s.vacant = msg.get()->vacant;
        ds.push_back(s);

        // map_merger::TransformPoint point;
        // point.request.point.src_robot = "robot_0";
        // point.request.point.x = s.x;
        // point.request.point.y = s.y;

        // ROS_ERROR("\e[1;34mCalling: %s\e[0m", sc_trasform.getService().c_str());
        // sc_trasform.call(point);
    }

    // update charging likelihood
    update_l1();
}

void docking::cb_auction(const adhoc_communication::EmAuction::ConstPtr &msg)
{
    //ROS_ERROR("\n\t\e[1;34mBid received: (%d, %d)\e[0m", msg.get()->robot, msg.get()->auction);

    // TODO
    /*
    // set auction id
    if(id > auction_id) // it is a new action from another robot, respond
        auction_id = id;
    else if(id > 0){    // it is an old auction from another robot, ignore
        ROS_ERROR("Bad auction ID, it should be greater than %d!", auction_id);
        return false;
    }
    else                // it is a new auction by this robot
        ; //++auction_id; //TODO //F
    */
    if (msg.get()->docking_station != best_ds.id)
    {
        /* Robot received a bid of an auction whose auctioned docking station is not the one the robot is interested in
         * at the moment, so it won't participate to the auction */
        ;  // ROS_INFO("Robot has no interested in participating to this auction");
    }
    else
    {
        /* The robot is interested in participating to the auction */
        participating_to_auction++;

        /* Start timer to force the robot to consider the auction concluded after some time, even in case it does not
         * receive the result of the auction.
         * To keep the timer active, we must store it on the heap; the timer is just pushed back in vector 'timers', which is cleared when no auction is pending (and so no timer is active). A better management should be possible for instance using insert(), at(), etc., but in previous tries the node always crash when using these functions... */
        ros::Timer timer = nh.createTimer(ros::Duration(FORCED_AUCTION_END_TIMEOUT),
                                          &docking::end_auction_participation_timer_callback, this, true, false);
        timer.start();
        timers.push_back(timer);  // TODO instead of recreating the timer, just reset it (but it is possible in real exp?)

        // if(-remaining_time > bid) //TODO
        {
            // ROS_INFO("The robot can bet an higher bid than the one received, so it will participate to the auction");
            adhoc_communication::SendEmAuction srv;
            // srv.request.dst_robot = id;
            srv.request.topic = "adhoc_communication/send_em_auction/reply";
            srv.request.auction.auction = msg.get()->auction;
            srv.request.auction.robot = robot_id;
            srv.request.auction.docking_station = best_ds.id;
            srv.request.auction.bid = get_llh();
            ROS_DEBUG("Calling service: %s", sc_send_auction.getService().c_str());
            sc_send_auction.call(srv);
        }
        // TODO
        /*
        else {
            ROS_ERROR("\n\t\e[1;34mIbut i have no chance to win... \e[0m");
            participating_to_auction--;
        }
        */
    }

    // TODO multi-hoop??
}

void docking::cb_auction_reply(const adhoc_communication::EmAuction::ConstPtr &msg)
{
    //ROS_INFO("Received reply to auction");

    // TODO //F probably in this version doesn't work with multi-hoop replies!!!!!
    //ROS_ERROR("\n\t\e[1;34mReply received: (%d, %d)\e[0m", msg.get()->robot, msg.get()->auction);
    // if(msg.get()->robot == robot_id) {

    std::vector<auction_bid_t>::iterator it = auction_bids.begin();
    for (; it != auction_bids.end(); it++)
        if ((*it).robot_id == msg.get()->robot)
        {
            // ROS_ERROR("\n\t\e[1;34mBut I have already received it...\e[0m");
            return;
        }

    // if(msg.get()->robot == robot_id && msg.get()->auction == auction_id)
    {
        // TODO it is ok ONLY assuming taht a robot during different auctions do not changes its bid!!!

        // ROS_ERROR("\n\t\e[1;34mReceived back my own auction... store bid of the sending robot\e[0m");

        if (managing_auction)
        {
            // ROS_ERROR("\n\t\e[1;34mreply stored\e[0m");
            auction_bid_t bid;
            bid.robot_id = msg.get()->robot;
            bid.bid = msg.get()->bid;
            auction_bids.push_back(bid);
        }
        else
        {
            // ROS_ERROR("\n\t\e[1;34mbut it was out of time...\e[0m");
            return;  // F ???
        }
    }
    // else
    //    ROS_ERROR("\n\t\e[1;34mbut it is not a reply to an auction of mine...\e[0m");

    /*
    // Multi-hoop reply
    adhoc_communication::SendEmAuction srv;
    srv.request.topic = "adhoc_communication/send_em_auction/reply";
    srv.request.auction.auction = msg.get()->auction;
    srv.request.auction.robot = msg.get()->robot;
    srv.request.auction.docking_station = msg.get()->docking_station;
    srv.request.auction.bid = msg.get()->bid;
    ROS_ERROR("\n\t\e[1;34m Multi-hoop send reply\e[0m");
    sc_send_auction.call(srv);
    */
}

void docking::end_auction_participation_timer_callback(const ros::TimerEvent &event)
{
    // ROS_INFO("Force to consider auction concluded");
    participating_to_auction--;
}

void docking::timerCallback(const ros::TimerEvent &event)
{
    /* The auction is concluded; the robot that started it has to compute who is the winner and must inform all the
     * other robots */
    ROS_INFO("Auction timeout: compute auction winner...");

    // ??? //TODO
    managing_auction = false;

    /* Compute auction winner: loop through all the received bids and find the robot that sent the highest one */
    int winner;
    float winner_bid = numeric_limits<int>::min();
    std::vector<auction_bid_t>::iterator it = auction_bids.begin();
    for (; it != auction_bids.end(); it++)
    {
        ROS_DEBUG("Robot %d placed %f", (*it).robot_id, (*it).bid);
        if ((*it).bid > winner_bid)
        {
            winner = (*it).robot_id;
            winner_bid = (*it).bid;
        }
    }

    // TODO
    
    if (auction_bids.size() < 3 && ds.size() == 1)
        ROS_ERROR("\n\t\e[1;34m OH NO!!!!!!!!!!!!!!!!!!!!!!!!!!!!! \e[0m");

    ROS_DEBUG("The winner is robot %d", winner);

    /* Delete stored bids to be able to start another auction in the future */
    auction_bids.clear();  // TODO inefficient!!!!

    /* Check if the robot that started the auction is the winner */
    if (winner == robot_id)
    {
        /* The robot won its own auction */
        ROS_ERROR("Winner of the auction"); //TODO specify which auction
        next_target_ds = best_ds;

        auction_winner = true;
        lost_other_robot_auction = false;

        timer_restart_auction.stop();  //TODO i'm not sure that this follows the idea in the paper...
    }
    else
    {
        /* The robot lost its own auction */
        ROS_ERROR("Robot lost its own auction");
        auction_winner = false;
        
        //TODO do better
        //TODO check also where this timer is stopped...
        /* If a robot is already in queue, the rescheduling timer must be started now (whereas if the robot is not already in queue, it should reach the queue before starting this timer) */
        if(robot_state == in_queue) {
            timer_restart_auction.setPeriod(ros::Duration(AUCTION_RESCHEDULING_TIME), true);
            timer_restart_auction.start();
        }
    }

    adhoc_communication::SendEmAuction srv_mgs;
    srv_mgs.request.topic = "adhoc_communication/auction_winner";
    srv_mgs.request.auction.auction = auction_id;
    srv_mgs.request.auction.robot = winner;

    srv_mgs.request.auction.docking_station = best_ds.id;
    srv_mgs.request.auction.bid = get_llh();

    // ROS_ERROR("\n\t\e[1;34m%s\e[0m", sc_send_auction.getService().c_str());
    sc_send_auction.call(srv_mgs);

    /* Computation completed */
    participating_to_auction--;
    update_state_required = true;
}

void docking::cb_charging_completed(const std_msgs::Empty &msg)
{
    adhoc_communication::SendEmDockingStation srv_msg;
    srv_msg.request.topic = "adhoc_communication/ds_state_update";
    srv_msg.request.docking_station.id = best_ds.id;
    srv_msg.request.docking_station.vacant = true;

    sc_send_docking_station.call(srv_msg);

    ds[target_ds.id].vacant = true; //TODO NO!!! we are not sure that DS are inserted in order of ID!!!!
}

void docking::timer_callback_schedure_auction_restarting(const ros::TimerEvent &event)
{
    ROS_INFO("Periodic re-auctioning");
    start_new_auction();
}

void docking::start_new_auction() {
    ROS_INFO("Starting new auction");
    
    /* The robot is starting an auction */
    managing_auction = true;
    auction_id++;
    participating_to_auction++;

    /* Start auction timer to be notified of auction conclusion */
    timer_finish_auction.setPeriod(ros::Duration(AUCTION_TIMEOUT), true);
    timer_finish_auction.start();

    /* Send broadcast message to inform all robots of the new auction */
    adhoc_communication::SendEmAuction srv;
    srv.request.topic = "adhoc_communication/send_em_auction/new_auction";
    srv.request.auction.auction = auction_id;
    srv.request.auction.robot = robot_id;
    srv.request.auction.docking_station = best_ds.id;
    srv.request.auction.bid = get_llh();
    ROS_DEBUG("Calling service: %s", sc_send_auction.getService().c_str());
    sc_send_auction.call(srv);

    /* Keep track of robot bid */
    auction_bid_t bid;
    bid.robot_id = robot_id;
    bid.bid = get_llh();
    auction_bids.push_back(bid);
}

void docking::cb_translate(const adhoc_communication::EmDockingStation::ConstPtr &msg) //TODO
{
    if (robot_prefix == "/robot_1")  // TODO ...
    {
        // ROS_ERROR("\n\t\e[1;34mReply to translation\e[0m");
        map_merger::TransformPoint point;

        point.request.point.src_robot = "robot_0";
        point.request.point.x = msg.get()->x;
        point.request.point.y = msg.get()->y;
        // if(sc_trasform.call(point)) ROS_ERROR("\e[1;34mTransformation succeded:\n\t\tOriginal point: (%f,
        // %f)\n\t\tObtained point: (%f, %f)\e[0m",point.request.point.x, point.request.point.y, point.response.point.x,
        // point.response.point.y);
    }
}

void docking::cb_auction_result(const adhoc_communication::EmAuction::ConstPtr &msg)
{
    // TODO the robot must check for its participation to the auction!!!

    // ROS_INFO("Received result of auction ... //TODO complete!!

    /* Check if the robot is interested in the docking station that was object of the auction whose result has been just
     * received */
     //TODO use multicast instead!!!
    if (msg.get()->docking_station == best_ds.id)
    {
        // ROS_INFO("Received result of an auction to which the robot participated");

        /* Since the robot received the result of an auction to which it took part, explorer node must be informed of a
         * possible change in the robot state */
        update_state_required = true;

        /* Check if the robot is the winner of the auction */
        if (robot_id == msg.get()->robot)
        {
            /* The robot won the auction */
            ROS_ERROR("Winner of the auction started by another robot");
            auction_winner = true;

            next_target_ds = best_ds;  // TODO this is safe if the best_ds is not modified during auctions!!!
            // TODO and what if best_ds is updated just a moment before starting the auction??? it shoul be ok because
            // the if above would be false
            timer_restart_auction.stop();  // TODO  i'm not sure that this follows the idea in the paper... but probably
                                           // this is needed otherwise when i have many pendning auction and with a
                                           // timeout enough high, i could have an inifite loop of restarting
                                           // auctions... or I could but a control in the timer_callback!!!
        }
        else
        {
            /* The robot has lost an auction started by another robot (because the robot that starts an auction does not
             * receive the result of that auction with this callback */
            // TODO should check if the robto took part to the auction
            ROS_ERROR("Robot didn't win this auction started by another robot");
            auction_winner = false;
            lost_other_robot_auction = true;
        }
    }
    else
        ROS_DEBUG("Received result of an auction the robot was not interested in: ignoring");
}

void docking::translate_coordinates(double starting_x, double starting_y, double *relative_x, double *relative_y)
{
    *relative_x = starting_x - origin_absolute_x;
    *relative_y = starting_y - origin_absolute_y;
}

void docking::ds_state_update_callback(const adhoc_communication::EmDockingStation::ConstPtr &msg)
{
    ROS_INFO("New state for ds%d: %s", msg.get()->id, (msg.get()->vacant ? "occupied" : "vacant") );
    ds[msg.get()->id].vacant = msg.get()->vacant; //TODO NO!!! we are not sure that DS are inserted in order of ID!!!!
}

void docking::check_vacancy_callback(const adhoc_communication::EmDockingStation::ConstPtr &msg)
{
    ROS_INFO("Received request for vacancy check for "); //TODO complete
    
    /* If the request for vacancy check is not about the target DS of the robot, for sure the robot is not occupying it */
    if (msg.get()->id == target_ds.id)  // TODO at the beginning no robot has already target_ds set, since it is set
                                        // only after the end of all auctions!!!!!

        /* If the robot is going to or already charging, or if it is going to check already checking for vacancy, it is (or may be, or will be) occupying the DS */ 
        if (robot_state == charging || robot_state == going_charging || robot_state == going_checking_vacancy ||
            robot_state == checking_vacancy)
        {
        
            /* Print some debut text */
            if (robot_state == charging)
                ROS_ERROR("\n\t\e[1;34mI'm using that DS!!!!\e[0m");
            else if (robot_state == going_charging || robot_state == going_checking_vacancy ||
                     robot_state == checking_vacancy)
                ROS_ERROR("\n\t\e[1;34mI'm approachign that DS too!!!!\e[0m");
            
            /* Reply to the robot that asked for the check, telling it that the DS is occupied */
            adhoc_communication::SendEmDockingStation srv_msg;
            srv_msg.request.topic = "explorer/adhoc_communication/reply_for_vacancy";
            srv_msg.request.docking_station.id = target_ds.id;
            sc_send_docking_station.call(srv_msg);
            
        }
        else
            ROS_ERROR("\n\t\e[1;34m target ds, but currently not used by the robot \e[0m");
    else
        ROS_ERROR("\n\t\e[1;34m robot is not targetting that ds\e[0m");
}

void docking::update_robot_state() //TODO simplify here (auction management and repeated code)...
{
    /*
     * Check if:
     * - there are no more pending auctions: this is to avoid to communicate contradicting information to the explorer
     *   node about the next state of the robot, so it is better to wait that all the auctions have been finished and
     *   then make a "global analisys" of the sistuation;
     * - an update of the state is required, i.e., that at least one auction was recently performed and that the robot
     *   took part to it (i.e., that it placed a bid).
     *
     * Notice that it may happen that the energy_mgmt node thinks that an update of the state is required, but maybe it
     * is not necessary: it is the explorer node that has to make some ckecks and, only if necessary, update the state;
     * the energy_mgmt node just informs the explorer node that something has recently happened.
     */
    if (update_state_required && participating_to_auction == 0)
    {
        /* An update of the robot state is required and it can be performed now */
        ROS_INFO("Sending information to explorer node about the result of recent auctions");

        /* Create the empty message to be sent */
        std_msgs::Empty msg;

        /* If the robot is not the winner of the most recent auction, notify explorer.
         * Notice that for sure it took part to at least one auction, or 'update_state_required' would be false and we wouldn't be in the true-branch of the previous if. */
        if (need_to_charge && !auction_winner)
        {
            /* Since there are no more pending auction, we can update the DS that is targetted by the robot */
            target_ds = next_target_ds;

            //TODO this if very bad way to avoid that a robot that lost an auction that was not its own rescheduled auiction think that, since a robot that is waiting in queue may either loose it own rescheduled auction or loose the auction of another robot!!
            if(robot_state != in_queue) {
                /* Notify explorer node about the lost auction */
                pub_lost_own_auction.publish(msg);
            }
            else {
            //TODO very bad also here...
                timer_restart_auction.setPeriod(ros::Duration(AUCTION_RESCHEDULING_TIME), true); 
                timer_restart_auction.start();
            }
        }

        /* If the robot is the winner of at least one auction, notify explorer */
        else if (auction_winner)
        {
            if (robot_state != charging || robot_state != going_charging || robot_state != going_checking_vacancy ||
                robot_state != checking_vacancy)  // TODO really necessary??? i don't think so...
            {
                /* Since there are no more pending auction, we can update the DS that is targetted by the robot */
                target_ds = next_target_ds;  // TODO but what if the robot is already charging at a certain DS which is
                                             // different from next_target_ds ??????????
                
                /* Notify explorer node about the new target DS */            
                geometry_msgs::PointStamped msg1;
                msg1.point.x = target_ds.x;
                msg1.point.y = target_ds.y;
                pub_new_target_ds.publish(msg1);
                
                /* Notify explorer node about the victory */
                pub_won_auction.publish(msg);  // TODO it is important that this is after the other pub!!!!
            }

        /* If the robot has lost an auction that was not started by it, notify explorer (because if the robot was
         * recharging, it has to leave the docking station) */
        }
        else if (lost_other_robot_auction)
        {
            /* Since there are no more pending auction, we can update the DS that is targetted by the robot */
            target_ds = next_target_ds;
            
            /* Notify explorer node about the lost auction */
            pub_lost_other_robot_auction.publish(msg);
            
        }

        /* Reset all the variables that are used to keep information about the auctions results (i.e., about the next
         * robot state) */
        update_state_required = false;
        auction_winner = false;
        lost_other_robot_auction = false;
        timers.clear(); //TODO inefficient!!
    }
    else
    {
        /* Do nothing, just print some debug text */
        if (participating_to_auction > 0 && !update_state_required)
            ROS_DEBUG("There are still pending auctions, and moreover no update is necessary for the moment");
        else if (!update_state_required)
            ROS_DEBUG("No state update required");
        else if (participating_to_auction > 0)
            ROS_DEBUG("There are still pending auctions, cannot update robot state");
        else
            ROS_FATAL("ERROR: the number of pending auctions is negative: %d", participating_to_auction);
    }
}

void docking::initLogpath()
{
    /*
     * CREATE LOG PATH
     * Following code enables to write the output to a file
     * which is localized at the log_path
     */

    /*
        std::stringstream robot_number;
        robot_number << robot_id;
        std::string prefix = "/robot_";
        std::string robo_name = prefix.append(robot_number.str());

        log_path = log_path.append("/energy_mgmt");
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
        */
}

void docking::set_target_ds_vacant(bool vacant) {
    adhoc_communication::SendEmDockingStation srv_msg;
    srv_msg.request.topic = "adhoc_communication/ds_state_update";
    srv_msg.request.docking_station.id = target_ds.id;
    srv_msg.request.docking_station.vacant = true;
    sc_send_docking_station.call(srv_msg);
    
    std::vector<ds_t>::iterator it;
    for(it = ds.begin(); it != ds.end(); it++)
        if((*it).id == target_ds.id) {
            (*it).vacant = vacant;
            break;
        }
}

void docking::compute_MST(int graph[V][V])
{
     int parent[V]; // Array to store constructed MST
     int key[V];   // Key values used to pick minimum weight edge in cut
     bool mstSet[V];  // To represent set of vertices not yet included in MST
 
     // Initialize all keys as INFINITE
     for (int i = 0; i < V; i++)
        key[i] = INT_MAX, mstSet[i] = false;
 
     // Always include first 1st vertex in MST.
     key[0] = 0;     // Make key 0 so that this vertex is picked as first vertex
     parent[0] = -1; // First node is always root of MST 
 
     // The MST will have V vertices
     for (int count = 0; count < V-1; count++)
     {
        // Pick the minimum key vertex from the set of vertices
        // not yet included in MST
        int u = minKey(key, mstSet);
 
        // Add the picked vertex to the MST Set
        mstSet[u] = true;
 
        // Update key value and parent index of the adjacent vertices of
        // the picked vertex. Consider only those vertices which are not yet
        // included in MST
        for (int v = 0; v < V; v++)
 
           // graph[u][v] is non zero only for adjacent vertices of m
           // mstSet[v] is false for vertices not yet included in MST
           // Update the key only if graph[u][v] is smaller than key[v]
          if (graph[u][v] && mstSet[v] == false && graph[u][v] <  key[v])
             parent[v]  = u, key[v] = graph[u][v];
     }
 
     // print the constructed MST
     printMST(parent, V, graph);
     
     
    
    for(int i=0; i<V; i++)
        for(int j=0; j<V; j++)
            mst[i][j] = 0;
            
    //TODO does not work if a DS is not connected to any other DS
    for(int i=1; i<V; i++) {
        mst[i][parent[i]] = 1; //parent[i] is the node closest to node i
        mst[parent[i]][i] = 1;
    }
    
    for(int i=0; i<V; i++) {
        for(int j=0; j<V; j++)
            ROS_ERROR("(%d, %d): %d ", i, j, mst[i][j]);         
    }
    
    int k = 0; //index of the closest recheable DS
    int target = 4; //the id of the DS that we want to reach
    std::vector<int> path; //sequence of nodes that from target leads to k; i.e., if the vector is traversed in the inverse order (from end to begin), it contains the path to go from k to target
    
    find_path(mst, k, target, path, -1);
    path.push_back(k);
    
    std::vector<int>::iterator it;
    for(it = path.begin(); it != path.end(); it++)
        ROS_ERROR("%d - ", *it);
    
}

// A utility function to find the vertex with minimum key value, from
// the set of vertices not yet included in MST
int docking::minKey(int key[], bool mstSet[])
{
   // Initialize min value
   int min = INT_MAX, min_index;
 
   for (int v = 0; v < V; v++)
     if (mstSet[v] == false && key[v] < min)
         min = key[v], min_index = v;
 
   return min_index;
}
 
// A utility function to print the constructed MST stored in parent[]
int docking::printMST(int parent[], int n, int graph[V][V])
{
   printf("Edge   Weight\n");
   for (int i = 1; i < V; i++)
      ROS_ERROR("%d - %d    %d \n", parent[i], i, graph[i][parent[i]]);
}

bool docking::find_path(int mst[][V], int start, int target, std::vector<int> &path, int prev_node) {
    for(int j=0; j < V; j++) 
        if(mst[start][j] == 1 && j != prev_node) {
            if(j == target || find_path(mst, j, target, path, start)) {
                path.push_back(j);
                return true; 
            }
        }
     return false;
}

void docking::moving_along_path_callback(std_msgs::Empty msg) {
    moving_along_path = false;
}
