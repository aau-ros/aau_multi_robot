#include <battery_simulate.h>

#include "data_logger/data_logger.h"

using namespace std;

//TODO(minor) comments, debugs, and so on...

battery_simulate::battery_simulate() //TODO the constructor should require as argument an instance of TimeManagerInterface
{
    loadParameters();
    initializeRobotName();
    DataLogger dt;
    dt.createLogFile(); //TODO
    battery_state_updater = new Computer2(&state);
}

void battery_simulate::loadParameters() {
    ros::NodeHandle nh_tilde("~");
    nh_tilde.param<std::string>("log_path", log_path, ""); //TODO getParam or param?
    nh_tilde.param<string>("robot_prefix", robot_prefix, "");
}

void battery_simulate::updateBatteryState() {
    robot_state_manager.getRobotState()->accept(battery_state_updater);
}

void battery_simulate::initializeRobotName() {
    if (robot_prefix.empty())
    {
        // Empty prefix: we are on hardware (i.e., real experiment)
        ROS_INFO("Real robot");

        // Set robot name and hostname
        char hostname[1024];
        hostname[1023] = '\0';
        gethostname(hostname, 1023);
        robot_name = string(hostname);

        // Set robot ID based on the robot name
        std::string bob = "bob";
        std::string marley = "marley";
        std::string turtlebot = "turtlebot";
        std::string joy = "joy";
        std::string hans = "hans";
        if (robot_name.compare(turtlebot) == 0)
            robot_id = 0;
        if (robot_name.compare(joy) == 0)
            robot_id = 1;
        if (robot_name.compare(marley) == 0)
            robot_id = 2;
        if (robot_name.compare(bob) == 0)
            robot_id = 3;
        if (robot_name.compare(hans) == 0)
            robot_id = 4;
    }
    else
    {
        // Prefix is set: we are in simulation
        ROS_INFO("Simulation");
        robot_name = robot_prefix;
    }
}

void battery_simulate::initializeSimulationTime() {
//    if(time_manager == NULL)
//        ROS_ERROR("Instance of TimeManager not set!");
//    else
//        time_last = time_manager->simulationTimeNow();
}

void battery_simulate::logBatteryState()
{
    std::string state_std;
//    if(initializing)
//        state_std = "initializing";
//    else if(idle_mode)
//        state_std = "idle";
//    else if(state.charging) 
//        state_std = "charging";
//    else if(advanced_computations_bool)
//        state_std = "computing";
//    else if(speed_linear > 0 || speed_angular > 0)
//        state_std = "moving";
//    else if(do_not_consume_battery)
//        state_std = "at_ds_for_computation";
//    else
//        state_std = "standing";
        
//    ros::Duration sim_time = time_manager->simulationTimeNow() - sim_time_start;
//    ros::WallDuration wall_time = ros::WallTime::now() - wall_time_start;

    //TODO    
//    battery_state_fs.open(battery_state_filename.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
//    battery_state_fs << sim_time.toSec() << "," << wall_time.toSec() << "," << state.remaining_time_run << "," << state.remaining_time_charge << "," << state.remaining_distance << "," << state_std << "," << state.consumed_energy_A << "," << state.consumed_energy_B << "," 
////    << last_traveled_distance << "," << total_traveled_distance << "," << time_manager->simulationTimeNow() << "," << ros::WallTime::now() 
//    << std::endl;
//    battery_state_fs.close();
}

void battery_simulate::publishBatteryState()
{
    pub_battery.publish(state);
}

void battery_simulate::run() {
    while(ros::ok()) {
        logBatteryState();
        updateBatteryState();
        publishBatteryState();
        ros::Duration(1).sleep(); //TODO(minor) rates???
        ros::spinOnce();
    }
}

void battery_simulate::setTimeManager(TimeManagerInterface *time_manager) {
    this->time_manager = time_manager;
}

void battery_simulate::createLogDirectory() {
    /* Create directory */
    log_path = log_path.append("/energy_mgmt");
    log_path = log_path.append(robot_name);
    boost::filesystem::path boost_log_path(log_path.c_str());
    if (!boost::filesystem::exists(boost_log_path))
    {
        ROS_INFO("Creating directory %s", log_path.c_str());
        try
        {
            if (!boost::filesystem::create_directories(boost_log_path))
            {
                ROS_ERROR("Cannot create directory %s: aborting node...", log_path.c_str());
                exit(-1);
            }
        }
        catch (const boost::filesystem::filesystem_error &e)
        {
            ROS_ERROR("Cannot create path %saborting node...", log_path.c_str());
            exit(-1);
        }
    }
    else
    {
        ROS_INFO("Directory %s already exists: log files will be saved there", log_path.c_str());
    }
}

void battery_simulate::createLogFiles() {
    /* Create file names */
    log_path = log_path.append("/");
    info_file = log_path + std::string("metadata_battery.csv");
    battery_state_filename = log_path + std::string("battery_state.csv");

//    fs_info.open(info_file.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
//    fs_info << "#power_sonar, power_laser, power_basic_computations, power_advanced_computations, power_microcontroller, power_moving_fixed_cost, power_per_speed, power_charging,max_linear_speed,initial_speed_avg" << std::endl;
//    fs_info << power_sonar << "," << power_laser << "," << power_basic_computations << "," << power_advanced_computations << "," << power_microcontroller << "," << power_moving_fixed_cost << "," << power_per_speed << "," << power_charging << "," << max_speed_linear << "," << speed_avg_init << std::endl;
//    fs_info.close();
    
//    battery_state_fs.open(battery_state_filename.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
//    battery_state_fs << "#elapsed_sim_time,elapsed_wall_time,state.remaining_time_run,state.remaining_time_charge,state.remaining_distance,state,consumed_energy_A,consumed_energy_B,last_traveled_distance,total_traveled_distance,sim_time,wall_time" << std::endl;
//    battery_state_fs.close();
    
//    sim_time_start = ros::Time::now();
//    wall_time_start = ros::WallTime::now();
}







/*************************
 ** Debugging functions **
 *************************/
//void battery_simulate::set_last_time() {
////    time_last = time_manager->simulationTimeNow();
//}

//double battery_simulate::last_time_secs() {
////    return time_last.toSec();
//}

//double battery_simulate::getElapsedTime() {
////    return elapsed_time;
//}

//void battery_simulate::spinOnce() {
//    ros::spinOnce();
//}

//double battery_simulate::getConsumedEnergyA() {
//    return state.consumed_energy_A;
//}

//double battery_simulate::getConsumedEnergyB() {
//    return state.consumed_energy_B;
//}

//double battery_simulate::getRemainingDistance() {
//    return state.remaining_distance;
//}

//double battery_simulate::getMaximumTravelingDistance() {
////    return maximum_traveling_distance;
//}

//double battery_simulate::getTotalTraveledDistance() {
////    return total_traveled_distance;
//}
