#include <battery_simulate.h>

//TODO(minor) comments, debugs, and so on...

using namespace std;

battery_simulate::battery_simulate() //TODO the constructor should require as argument an instance of TimeManagerInterface
{
    ROS_INFO("Creating instance of battery_simulate");
    loadParameters();
    initializeRobotName();

    pub_battery = nh.advertise<explorer::battery_state>("battery_state", 1);
    ROS_INFO("Instance created successfully");
}

void battery_simulate::loadParameters() {
    ros::NodeHandle nh_tilde("~");
    if(!nh_tilde.getParam("log_path", log_path))
        ROS_FATAL("INVALID PARAM");
    if(!nh_tilde.getParam("robot_prefix", robot_prefix))
        ROS_FATAL("INVALID PARAM");
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

void battery_simulate::logBatteryState()
{    
    ros::Duration sim_time = time_manager->simulationTimeNow() - sim_time_start;
    ros::WallDuration wall_time = ros::WallTime::now() - wall_time_start;  
    battery_state_fs.open(battery_state_filename.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
    battery_state_fs << sim_time.toSec() << "," << wall_time.toSec() << "," << state->remaining_time_run << "," << state->remaining_time_charge << "," << state->remaining_distance << "," << "," << state->consumed_energy_A << "," << state->consumed_energy_B << "," 
//    << last_traveled_distance << "," << total_traveled_distance << "," << time_manager->simulationTimeNow() << "," << ros::WallTime::now() 
    << std::endl;
    battery_state_fs.close();

    std::stringstream stream;
    stream << time_manager->simulationTimeNow() << "," << state->consumed_energy_A << "," << state->consumed_energy_B << std::endl;
    data_logger->updateLogFile("metadata.csv", stream);

}

void battery_simulate::updateBatteryState() {
    battery_state_updater->updateBatteryState();
}

void battery_simulate::setTimeManager(TimeManagerInterface *time_manager) {
    this->time_manager = time_manager;
}

void battery_simulate::setBatteryStateUpdater(BatteryStateUpdaterInterface *battery_state_updater) {
    this->battery_state_updater = battery_state_updater;
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
    battery_state_filename = log_path + std::string("battery_state.csv");
    
    battery_state_fs.open(battery_state_filename.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
    battery_state_fs << "#elapsed_sim_time,elapsed_wall_time,state.remaining_time_run,state.remaining_time_charge,state.remaining_distance,consumed_energy_A,consumed_energy_B,last_traveled_distance,total_traveled_distance,sim_time,wall_time" << std::endl;
    battery_state_fs.close();
    
    sim_time_start = ros::Time::now();
    wall_time_start = ros::WallTime::now();

    data_logger = new DataLogger("energy_mgmt_test", robot_prefix, log_path);
    std::stringstream stream;
    stream << "#..." << std::endl;
    data_logger->createLogFile("metadata.csv", stream);
}

void battery_simulate::publishBatteryState() {
    ROS_INFO("publishing battery state");
    pub_battery.publish(*state);
    ROS_INFO("%.1f, %.1f", state->remaining_distance, state->maximum_traveling_distance);
}

void battery_simulate::setBatteryState(explorer::battery_state *state) {
    this->state = state;
}
