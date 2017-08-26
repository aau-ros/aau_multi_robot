#include "docking_station_detector.h"

DockingStationDetector::DockingStationDetector() {
    ros::NodeHandle nh;
    std::string my_prefix = ""; //TODO
    sc_reachable_target = nh.serviceClient<explorer::DistanceFromRobot>(my_prefix + "explorer/reachable_target");
}

void DockingStationDetector::preloadDockingStations()
{
    ROS_INFO("Preload DSs and mark them as undiscovered");

    int index = 0;
    double x, y;

    // If the x-coordinate of a DS with index <index> is found, it means that that DS is present in the file and must be loaded. Notice that we assume that if there is a x-coordinate, also the corresponding y is present
    ros::NodeHandle private_nh("~");
    while (private_nh.hasParam("d" + SSTR(index) + "/x")) //TODO(minor) maybe ds would be nicer
    {
        // Load coordinates of the new DS
        private_nh.param("d" + SSTR(index) + "/x", x, 0.0);
        private_nh.param("d" + SSTR(index) + "/y", y, 0.0);

        //Store new DS
        ds_t new_ds;
        new_ds.id = index;
        new_ds.world_x = x, new_ds.world_y = y;
        new_ds.timestamp = ros::Time::now().toSec();
        abs_to_rel(x, y, &(new_ds.x), &(new_ds.y));
        new_ds.vacant = true;  // TODO(minor) param...
        undiscovered_dss.push_back(new_ds);

        // Delete the loaded parameters (since they are not used anymore)
        private_nh.deleteParam("d" + SSTR(index) + "/x");
        private_nh.deleteParam("d" + SSTR(index) + "/y");

        // Prepare to search for next DS (if it exists) in the file
        index++;
    }
    
    // Store the number of DSs in the environment
//    num_ds = undiscovered_dss.size(); //TODO(minor) a problem in general!!

    // Print loaded DSs with their coordinates relative to the local reference system of the robot, and also store them on file  // TODO(minor) or better in global system?
    ROS_FATAL("MISSING");
//    for (auto it = undiscovered_dss.begin(); it != undiscovered_dss.end(); it++) {
//        ROS_DEBUG("ds%d: (%f, %f)", (*it).id, (*it).x, (*it).y);
//        ds_fs.open(ds_filename.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
//        ds_fs << it->id << "," << it->x << "," << it->y << std::endl;
//        ds_fs.close();
//    }
}

void DockingStationDetector::detectNewDockingStations()
{   
    unsigned int i=0;
    for (auto it = undiscovered_dss.begin(); it != undiscovered_dss.end() && i < undiscovered_dss.size() && undiscovered_dss.size() > 0; it++, i++)
    {
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
            ROS_ERROR("Unable to check if %s is reachable, retrying later...", dsToStr(*it).c_str());
            continue;
        }

        if (reachable)
        {
            ROS_INFO("ds%d is now reachable", it->id);
            ROS_FATAL("MISSING");
//            docking_station_manager->addDockingStation(*it);

            ROS_DEBUG("Erase at position %u; size after erase is %u", i, (unsigned int)undiscovered_dss.size() - 1);
            undiscovered_dss.erase(undiscovered_dss.begin() + i);
            
//            it = undiscovered_dss.begin(); //since it seems that the pointer is invalidated after the erase, so better restart the check... (http://www.cplusplus.com/reference/vector/vector/erase/) //TODO
//            i=0;
            break;            
        }
        else
            ROS_DEBUG("ds%d is not reachable at the moment: ", it->id);
    }  
}

std::string DockingStationDetector::dsToStr(ds_t ds) { //TODO macro? //TODO put also in other classes (or in a common .h)
    return "ds_" + SSTR(ds.id);
}

void DockingStationDetector::abs_to_rel(double absolute_x, double absolute_y, double *relative_x, double *relative_y)
{
    ROS_FATAL("MISSING");
    // Use these if the /map frame origin coincides with the robot starting position in Stage (which should be the case)
//    *relative_x = absolute_x - origin_absolute_x;
//    *relative_y = absolute_y - origin_absolute_y;

    // Use these if the /map frame origin coincides with Stage origin
//    *relative_x = absolute_x;
//    *relative_y = absolute_y;
}

void DockingStationDetector::rel_to_abs(double relative_x, double relative_y, double *absolute_x, double *absolute_y)
{
    ROS_FATAL("MISSING");
    // Use these if the /map frame origin coincides with the robot starting position in Stage (which should be the case)
//    *absolute_x = relative_x + origin_absolute_x;
//    *absolute_y = relative_y + origin_absolute_y;
    
    // Use these if the /map frame origin coincides with Stage origin
//    *absolute_x = relative_x;
//    *absolute_y = relative_y;
}
