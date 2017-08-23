#include "docking_station_manager.h"

DockingStationManager::DockingStationManager() {}

void DockingStationManager::getDockingStationIterator() {
    ROS_ERROR("invalidation"); //TODO how to handle the fact that iterator can be invalidated?
}

void docking::preload_docking_stations()
{
    ROS_INFO("Preload DSs and mark them as undiscovered");

    int index = 0;  // index of the DS, used to loop among all the docking stations
                    // inserted in the file
    double x, y;    // DS coordinates
    
    // TOSO should be reduntant, since the functions that read undiscovered_ds are in the same thread of this function...
//    boost::unique_lock< boost::shared_mutex > lock(ds_mutex);
    ds_mutex.lock();

    /* If the x-coordinate of a DS with index <index> is found, it means that that DS
     * is present in the file and must be loaded. Notice that we assume that if there is a x-coordinate, also the corresponding y is present */
    ros::NodeHandle nh_tilde("~");
    while (nh_tilde.hasParam("d" + SSTR(index) + "/x")) //TODO(minor) maybe ds would be nicer
    {
        /* Load coordinates of the new DS */
        nh_tilde.param("d" + SSTR(index) + "/x", x, 0.0);
        nh_tilde.param("d" + SSTR(index) + "/y", y, 0.0);

        /* Store new DS */
        ds_t new_ds;
        new_ds.id = index;
        new_ds.world_x = x, new_ds.world_y = y;
        new_ds.timestamp = ros::Time::now().toSec();
        abs_to_rel(x, y, &(new_ds.x), &(new_ds.y));
        new_ds.vacant = true;  // TODO(minor) param...
        undiscovered_ds.push_back(new_ds);

        /* Delete the loaded parameters (since they are not used anymore) */
        nh_tilde.deleteParam("d" + SSTR(index) + "/x");
        nh_tilde.deleteParam("d" + SSTR(index) + "/y");

        /* Prepare to search for next DS (if it exists) in the file */
        index++;
    }
    
    /* Store the number of DSs in the environment */
    num_ds = undiscovered_ds.size(); //TODO(minor) a problem in general!!

    /* Print loaded DSs with their coordinates relative to the local reference system of the robot, and also store them on file
     */  // TODO(minor) or better in global system?
    for (std::vector<ds_t>::iterator it = undiscovered_ds.begin(); it != undiscovered_ds.end(); it++) {
        ROS_DEBUG("ds%d: (%f, %f)", (*it).id, (*it).x, (*it).y);
        ds_fs.open(ds_filename.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
        ds_fs << it->id << "," << it->x << "," << it->y << std::endl;
        ds_fs.close();
    }
        
    // TODO(minor) vary bad if we don't know the number of DS a-priori
    /* Initialize docking station graph */
    for (int i = 0; i < num_ds; i++)
    {
        std::vector<int> temp;
        std::vector<float> temp_f;
        for (unsigned int j = 0; j < undiscovered_ds.size(); j++) {
            temp.push_back(-1);
            temp_f.push_back(-1);
        }
        ds_mst.push_back(temp);
        ds_graph.push_back(temp_f);
    }
    ds_mutex.unlock();
      
}

//DONE+
void docking::discover_docking_stations() //TODO(minor) comments
{
    ROS_INFO("discover_docking_stations");
    
//    boost::unique_lock< boost::shared_mutex > lock(ds_mutex); //TODO improve
    ds_mutex.lock();
    
    // Check if there are DSs that can be considered discovered (a DS is considered discovered if the euclidean distance between it and the robot is less than the range of the "simulated" fiducial signal emmitted by the DS
    for (std::vector<ds_t>::iterator it = undiscovered_ds.begin(); it != undiscovered_ds.end(); it++)
    {
        double dist = distance_from_robot((*it).x, (*it).y, true);
        if (dist < 0)
            ROS_WARN("Failed distance computation!");
        else if (dist < fiducial_signal_range)
        {
            // Safety check
            if(it->id < 0 || it->id >= num_ds) {
                log_major_error("inserting invalid DS id in discovered_ds!!!");
                   
            }
            else {
            
                // safety check: checks that DS is not already present in discovered_ds
                bool already_inserted = false;
                for(unsigned int i=0; i < discovered_ds.size() && !already_inserted; i++)
                    if(discovered_ds.at(i).id == it->id) {
                        log_major_error("this DS has been already inserted!!!"); //this should not happen!!!
                        already_inserted = true;
                    }
                        
                if(!already_inserted) {
                    // Store new DS in the vector of known DSs
                    ROS_INFO("Found new DS ds%d at (%f, %f). Currently, no path for this DS is known...", (*it).id, (*it).x,
                             (*it).y);  // TODO(minor) index make sense only in simulation (?, not sure...)
                    it->vacant = true; //TODO probably reduntant
                    it->timestamp = ros::Time::now().toSec();
                    
                    discovered_ds.push_back(*it); //TODO(minor) change vector name, from discovered_ds to unreachable_dss

                    // Inform other robots about the "new" DS
                    adhoc_communication::SendEmDockingStation send_ds_srv_msg;
                    send_ds_srv_msg.request.topic = "docking_stations";
                    send_ds_srv_msg.request.docking_station.id = (*it).id;
                    double x, y;
                    rel_to_abs((*it).x, (*it).y, &x, &y);
                    send_ds_srv_msg.request.docking_station.x = x;
                    send_ds_srv_msg.request.docking_station.y = y;
                    send_ds_srv_msg.request.docking_station.vacant = true;  // TODO(minor) sure???
                    send_ds_srv_msg.request.docking_station.header.timestamp = it->timestamp;
                    send_ds_srv_msg.request.docking_station.header.sender_robot = robot_id;
                    send_ds_srv_msg.request.docking_station.header.message_id = getAndUpdateMessageIdForTopic("docking_stations");
                    sc_send_docking_station.call(send_ds_srv_msg);
                }

                // Remove discovered DS from the vector undiscovered DSs
                undiscovered_ds.erase(it);
                
                // Since it seems that erase() makes the iterator invalid, it is better to stop the loop and retry later from the beginning of the vector with a new and aclean iterator //TODO or we could "clean" the current one, although it seems that Valgrind complains...
                
                break;
            }
            
        } else
            ROS_DEBUG_COND(LOG_TRUE, "ds%d has not been discovered yet", (*it).id);        
    }
    ds_mutex.unlock();
}

void docking::check_reachable_ds()
{
    ROS_INFO("check_reachable_ds");
//    boost::unique_lock< boost::shared_mutex > lock(ds_mutex);

    ds_mutex.lock();
    
    bool new_ds_discovered = false;
    unsigned int i=0;
    for (std::vector<ds_t>::iterator it = discovered_ds.begin(); it != discovered_ds.end() && i < discovered_ds.size() && discovered_ds.size() > 0; it++, i++)
    {
        /* If the DS is inside a fiducial laser range, it can be considered
         * discovered */
        bool reachable;
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
            ROS_ERROR("Unable to check if ds%d is reachable, retrying later...", (*it).id);
            break;
        }

        if (reachable)
        {
            ROS_INFO("ds%d is now reachable", (*it).id);
            
            it->timestamp = ros::Time::now().toSec();
            
            adhoc_communication::EmDockingStation new_ds_msg;
            new_ds_msg.id = it->id;
            new_ds_msg.x = it->x;
            new_ds_msg.y = it->y;
            new_ds_msg.header.timestamp = it->timestamp;
            //ROS_ERROR("publishing on %s", pub_new_ds_on_graph.getTopic().c_str());
            //TODO publishing here just to avoid to publish the message too early, but of course this is not a good place, and it is not necessary to publish the message every time
            //std_msgs::Int32 ds_count_msg;
            //ds_count_msg.data = num_ds;
            //ROS_ERROR("publishing on topic %s", pub_ds_count.getTopic().c_str());
            //pub_ds_count.publish(ds_count_msg);
            new_ds_msg.total_number_of_ds = num_ds;
            pub_new_ds_on_graph.publish(new_ds_msg);
            
            new_ds_discovered = true;
//            ds_t new_ds; 
//            new_ds.id = it->id;
            
            if(it->id < 0 || it->id >= num_ds) {
                log_major_error("Trying to insert in 'ds' an invalid DS!!!");
                break;
            }
            it->vacant = true; //although it should be reduntant
            
//            new_ds.x = it->x;
//            new_ds.y = it->y;
//            new_ds.vacant = it->vacant;
            
            bool already_inserted = false;
            for(unsigned int i2=0; i2 < ds.size() && !already_inserted; i2++)
                if(ds.at(i2).id == it->id) {
                    log_major_error("this DS has been already inserted!!!"); //this should not happen!!!
                    already_inserted = true;
                }
                    
            if(!already_inserted) {
            
                it->has_EOs = true;
                ds.push_back(*it);
            
            }
            
            
//            ds.push_back(new_ds);
            
//            int id1 = ds[ds.size()-1].id;
//            if(id1 != it->id)
//                log_major_error("error");
//            int id2 = it->id;
            //discovered_ds.erase(it);
            ROS_INFO("erase at position %d; size after delete is %u", i, (unsigned int)(discovered_ds.size() - 1));
            discovered_ds.erase(discovered_ds.begin() + i);
            
//            it = discovered_ds.begin(); //since it seems that the pointer is invalidated after the erase, so better restart the check... (http://www.cplusplus.com/reference/vector/vector/erase/)
//            i=0;
            break;
            
            
//            // Visualize in RViz
//            visualization_msgs::Marker marker;

//            marker.header.frame_id = robot_prefix + "/map";
//            marker.header.stamp = ros::Time::now();
//            marker.header.seq = it->id;
//            marker.ns = "ds_position";
//            marker.id = it->id;
//            marker.type = visualization_msgs::Marker::SPHERE;
//            marker.action = visualization_msgs::Marker::ADD;
//            //marker.lifetime = ros::Duration(10); //TODO //F
//            marker.scale.x = 0.5;
//            marker.scale.y = 0.5;
//            marker.scale.z = 0.5;
//            marker.pose.position.x = it->x;
//            marker.pose.position.y = it->y;
//            marker.pose.position.z = 0;
//            marker.pose.orientation.x = 0.0;
//            marker.pose.orientation.y = 0.0;
//            marker.pose.orientation.z = 0.0;
//            marker.pose.orientation.w = 1.0;
//            marker.color.a = 1.0;
//            marker.color.r = 0.0;
//            marker.color.g = 0.0;
//            marker.color.b = 1.0;
//            pub_ds_position.publish< visualization_msgs::Marker >(marker);
//            pub_ds_position.publish< visualization_msgs::Marker >(marker);
//            pub_ds_position.publish< visualization_msgs::Marker >(marker);
//            pub_ds_position.publish< visualization_msgs::Marker >(marker);
      
            //ROS_ERROR("x: %.1f, y: %.1f", pose->pose.pose.position.x, pose->pose.pose.position.y);
            

//            if(id1 != id2)
//                log_major_error("ERROR");
            
        }
        else
            ROS_DEBUG("ds%d is not reachable at the moment: ", (*it).id);
    }

//    if (new_ds_discovered || recompute_graph)
    if (new_ds_discovered)
//    {
        update_ds_graph();
//        
////        int start = ds.at(0).id;
////        int end = ds.at(ds.size()-1).id;
////        find_path_2(start, end, path);

//        // construct MST starting from ds graph
//        //compute_MST_2(1);
//        
//    }
    
    ds_mutex.unlock();
    
    ROS_INFO("end check_reachable_ds()");
    
}

bool docking::optimal_ds_is_set() {
    return optimal_ds_set;
}

bool docking::set_optimal_ds(int id) {
//    boost::shared_lock< boost::shared_mutex > lock(ds_mutex);

    old_optimal_ds_id = optimal_ds_id;
    optimal_ds_id = id;
    
    for(unsigned int i=0; i < ds.size(); i++)
        if(ds.at(i).id == id) {
            optimal_ds_x = ds.at(i).x;
            optimal_ds_y = ds.at(i).y;
            optimal_ds_timestamp = ds.at(i).timestamp;
            break;
        }
        
    if (optimal_ds_set)
        ROS_INFO("Change optimal DS: ds%d -> ds%d", old_optimal_ds_id, optimal_ds_id);
    else
        ROS_INFO("Change optimal DS: (none) -> ds%d", optimal_ds_id);
        
    optimal_ds_set = true;
    send_optimal_ds();
    return true;
}

double docking::get_optimal_ds_timestamp() {
    return optimal_ds_timestamp;
}

void docking::free_ds(int id) {
    ROS_DEBUG("freeing ds%d", id);

    if(id < 0 || id >= num_ds) {
        log_major_error("invalid ds in free_ds!");
        ROS_INFO("id is %d", id);
        return;
    }

    adhoc_communication::SendEmDockingStation srv_msg;
    srv_msg.request.topic = "docking_stations";
    srv_msg.request.dst_robot = group_name;
    srv_msg.request.docking_station.id = id;
    double x, y, timestamp;
    
    for(unsigned int i=0; i < ds.size(); i++)
        if(ds.at(i).id == id) {
            ds.at(i).vacant = vacant;
            ds.at(i).timestamp = ros::Time::now().toSec();
            timestamp = ds.at(i).timestamp;
            rel_to_abs(ds.at(i).x, ds.at(i).y, &x, &y);
            break;
        }
    
    srv_msg.request.docking_station.x = x;  // it is necessary to fill also this fields because when a Ds is
                                            // received, robots perform checks on the coordinates
    srv_msg.request.docking_station.y = y;
    srv_msg.request.docking_station.vacant = true;

    srv_msg.request.docking_station.header.timestamp = timestamp;
    srv_msg.request.docking_station.header.sender_robot = robot_id;
    srv_msg.request.docking_station.header.message_id = getAndUpdateMessageIdForTopic("docking_stations");
    
    sc_send_docking_station.call(srv_msg);

    ROS_INFO("Updated own information about ds%d state (%s -> %s)", get_optimal_ds_id(), "occupied", "vacant");

}

void docking::set_optimal_ds_vacant(bool vacant)
{
//    if(get_target_ds_id() < 0 || get_target_ds_id() >= num_ds) {
//        log_major_error("invalid target_ds in set_vacant!");
//        return;
//    }

    for(unsigned int i=0; i < ds.size(); i++)
        if(ds.at(i).id == get_optimal_ds_id()) {
            ds.at(i).vacant = vacant;
            ds.at(i).timestamp = ros::Time::now().toSec();   
        }

    adhoc_communication::SendEmDockingStation srv_msg;
    srv_msg.request.topic = "docking_stations";
    srv_msg.request.dst_robot = group_name;
    srv_msg.request.docking_station.id = get_optimal_ds_id();
    double x, y;
    
    rel_to_abs(get_optimal_ds_x(), get_optimal_ds_y(), &x, &y);
    
    srv_msg.request.docking_station.x = x;  // it is necessary to fill also this fields because when a DS is
                                            // received, robots perform checks on the coordinates
    srv_msg.request.docking_station.y = y;
    srv_msg.request.docking_station.vacant = vacant;
    srv_msg.request.docking_station.header.timestamp = get_optimal_ds_timestamp();
    srv_msg.request.docking_station.header.sender_robot = robot_id;
    srv_msg.request.docking_station.header.message_id = getAndUpdateMessageIdForTopic("docking_stations");
    sc_send_docking_station.call(srv_msg);

    ROS_INFO("Updated own information about ds%d state (%s -> %s)", get_optimal_ds_id(), (vacant ? "occupied" : "vacant"), (vacant ? "vacant" : "occupied"));

}

void setOptimalDockingStation() {
    ...
    log_optimal_ds();
}

void getOptimalDockingStation() {
//    return ...;
}

void docking::log_optimal_ds() {
    ROS_INFO("logging");
//    optimal_ds_mutex.lock();
    /* Keep track of the optimal and target DSs in log file */
//    if(old_optimal_ds_id_for_log != get_optimal_ds_id() || old_target_ds_id_for_log != get_target_ds_id() ) {
    if(old_optimal_ds_id_for_log != get_optimal_ds_id() ) {
        ros::Duration time = ros::Time::now() - time_start;
        fs_csv.open(csv_file.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
//        ROS_DEBUG("%d", target_ds_id);
        fs_csv << ros::Time::now().toSec() << "," << ros::WallTime::now().toSec() << ","
            << get_optimal_ds_id() << std::endl;
        fs_csv.close();
        old_optimal_ds_id_for_log = get_optimal_ds_id();
//        old_target_ds_id_for_log = get_target_ds_id();
    }
//    optimal_ds_mutex.unlock();
}
