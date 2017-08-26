//void DockingStationManager::discoverDockingStations()
//{   
//    // Check if there are DSs that can be considered discovered (a DS is considered discovered if the euclidean distance between it and the robot is less than the range of the "simulated" fiducial signal emmitted by the DS
//    for (auto it = undiscovered_ds.begin(); it != undiscovered_ds.end(); it++)
//    {
//        double dist = distance_from_robot((*it).x, (*it).y, true);
//        if (dist < 0)
//            ROS_WARN("Failed distance computation!");
//        else if (dist < fiducial_signal_range)
//        {
//            // Safety check
//            if(it->id < 0 || it->id >= num_ds) {
//                log_major_error("inserting invalid DS id in discovered_ds!!!");
//                   
//            }
//            else {
//            
//                // safety check: checks that DS is not already present in discovered_ds
//                bool already_inserted = false;
//                for(unsigned int i=0; i < discovered_ds.size() && !already_inserted; i++)
//                    if(discovered_ds.at(i).id == it->id) {
//                        log_major_error("this DS has been already inserted!!!"); //this should not happen!!!
//                        already_inserted = true;
//                    }
//                        
//                if(!already_inserted) {
//                    // Store new DS in the vector of known DSs
//                    ROS_INFO("Found new DS ds%d at (%f, %f). Currently, no path for this DS is known...", (*it).id, (*it).x,
//                             (*it).y);  // TODO(minor) index make sense only in simulation (?, not sure...)
//                    it->vacant = true; //TODO probably reduntant
//                    it->timestamp = ros::Time::now().toSec();
//                    
//                    discovered_ds.push_back(*it); //TODO(minor) change vector name, from discovered_ds to unreachable_dss

//                    // Inform other robots about the "new" DS
//                    adhoc_communication::SendEmDockingStation send_ds_srv_msg;
//                    send_ds_srv_msg.request.topic = "docking_stations";
//                    send_ds_srv_msg.request.docking_station.id = (*it).id;
//                    double x, y;
//                    rel_to_abs((*it).x, (*it).y, &x, &y);
//                    send_ds_srv_msg.request.docking_station.x = x;
//                    send_ds_srv_msg.request.docking_station.y = y;
//                    send_ds_srv_msg.request.docking_station.vacant = true;  // TODO(minor) sure???
//                    send_ds_srv_msg.request.docking_station.header.timestamp = it->timestamp;
//                    send_ds_srv_msg.request.docking_station.header.sender_robot = robot_id;
//                    send_ds_srv_msg.request.docking_station.header.message_id = getAndUpdateMessageIdForTopic("docking_stations");
//                    sc_send_docking_station.call(send_ds_srv_msg);
//                }

//                // Remove discovered DS from the vector undiscovered DSs
//                undiscovered_ds.erase(it);
//                
//                // Since it seems that erase() makes the iterator invalid, it is better to stop the loop and retry later from the beginning of the vector with a new and aclean iterator //TODO or we could "clean" the current one, although it seems that Valgrind complains...
//                
//                break;
//            }
//            
//        } else
//            ROS_DEBUG_COND(LOG_TRUE, "ds%d has not been discovered yet", (*it).id);        
//    }
//    ds_mutex.unlock();
//}
