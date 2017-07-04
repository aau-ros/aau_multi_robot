#include <battery_simulate.h>

#define INFINITE_ENERGY false

using namespace std;

//TODO(minor) comments, debugs, and so on...

battery_simulate::battery_simulate() //TODO the constructor should require as argument an instance of TimeManagerInterface
{
    // read parameters
    nh.getParam("energy_mgmt/speed_avg", speed_avg_init);
    nh.getParam("energy_mgmt/power_charging", power_charging); // W (i.e, watt)
    nh.getParam("energy_mgmt/power_moving", power_moving);     // W/(m/s)
    nh.getParam("energy_mgmt/power_standing", power_standing); // W
    nh.getParam("energy_mgmt/power_idle", power_idle);         // W
    nh.getParam("energy_mgmt/power_basic_computations", power_basic_computations);  // W
    nh.getParam("energy_mgmt/power_advanced_computations", power_advanced_computation);  // W
    nh.getParam("energy_mgmt/charge_max", charge_max);         // Wh (i.e, watt-hour) //TODO(minor) which value is good in the YAML file?
    nh.getParam("energy_mgmt/max_linear_speed", max_speed_linear); // m/s
    nh.getParam("energy_mgmt/mass", mass); // kg
    advanced_computations_bool = true;
    
    //ROS_ERROR("%f, %f, %f, %f", power_charging, power_moving, power_standing, charge_max);

    // initialize private variables
    speed_angular = 0;
    time_moving = 0;   //TODO(minor) useless?
    time_standing = 0; //TODO(minor) useless?
    perc_moving = 0.5; //TODO(minor) useless?
    perc_standing = 0.5; //TODO(minor) useless?
    output_shown = false; //TODO(minor) useless?
    speed_avg = speed_avg_init;
    idle_mode = false;
    do_not_consume_battery = false;
    
    charge = charge_max;
    total_energy = charge_max * 3600; // J (i.e, joule)       
    remaining_energy = total_energy;
    speed_linear = max_speed_linear;
    maximum_running_time = total_energy / (power_moving * max_speed_linear + power_standing + power_basic_computations + power_advanced_computation); // s //TODO power_advanced_computation is missing a final 's'
    
    //ROS_ERROR("maximum_running_time: %f", maximum_running_time);
    
    if(INFINITE_ENERGY) {
        total_energy = 10000;
        remaining_energy = 10000;
        power_standing = 0.0;  
        power_moving   = 0.0;  
        power_charging = 100.0;
        maximum_running_time = 100000000000000000;
    }

    // initialize battery state
    state.charging = false;
    state.soc = 1; // (adimensional) // TODO(minor) if we assume that the robot starts fully_charged
    state.remaining_time_charge = 0; // since the robot is assumed to be fully charged when the exploration starts
    state.remaining_time_run = maximum_running_time; //s //TODO(minor) "maximum" is misleading: use "estimated"...
    state.remaining_distance = maximum_running_time * speed_avg_init; //m //TODO(minor) explain why we use max_speed_linear and not min_speed, etc.

    // advertise topics
    pub_battery = nh.advertise<explorer::battery_state>("battery_state", 1);
    pub_charging_completed = nh.advertise<std_msgs::Empty>("charging_completed", 1);
    
    pub_full_battery_info = nh.advertise<explorer::battery_state>("full_battery_info", 1);

    // subscribe to topics
    sub_speed = nh.subscribe("avg_speed", 1, &battery_simulate::cb_speed, this);
    sub_cmd_vel = nh.subscribe("cmd_vel", 1, &battery_simulate::cb_cmd_vel, this);
    sub_robot = nh.subscribe("explorer/robot", 100, &battery_simulate::cb_robot, this);    
//    sub_time = nh.subscribe("totalTime", 1, &battery_simulate::totalTime, this); // TODO(minor) do we need this?

    //ROS_ERROR("remaining distance: %f", state.remaining_distance);
    pub_full_battery_info.publish(state);
    
    ros::NodeHandle nh_tilde("~");
    nh_tilde.param<std::string>("log_path", log_path, "");
    nh_tilde.param<string>("robot_prefix", robot_prefix, "");
    /* Initialize robot name */
    if (robot_prefix.empty())
    {
        /* Empty prefix: we are on an hardware platform (i.e., real experiment) */
        ROS_INFO("Real robot");

        /* Set robot name and hostname */
        char hostname[1024];
        hostname[1023] = '\0';
        gethostname(hostname, 1023);
        robot_name = string(hostname);

        /* Set robot ID based on the robot name */
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
        /* Prefix is set: we are in a simulation */
        ROS_INFO("Simulation");
        robot_name = robot_prefix;
    }
        
}

void battery_simulate::initializeSimulationTime() {
    if(time_manager == NULL)
        ROS_ERROR("Instance of TimeManager not set!");
    else
        time_last = time_manager->simulationTimeNow();
        
}

void battery_simulate::cb_robot(const adhoc_communication::EmRobot::ConstPtr &msg)
{
    if (msg.get()->state == charging)
    {
        ROS_DEBUG("Start recharging");
        state.charging = true;
    }
    else {
        /* The robot is not charging; if the battery was previously under charging, it means that the robot aborted the
        recharging process */
        if (state.charging == true)
        {
            ROS_DEBUG("Recharging aborted");
            state.charging = false;
        }
    }
    
    if(msg.get()->state == fully_charged) //when the robot is fully_charged, it is left a little bit at the DS to allow him to compute the next DS without consuming energy, so that the check of the reachability of frontiers also using the Ds graph makes sense
        do_not_consume_battery = true;
    else
        do_not_consume_battery = false;
    
    if(msg.get()->state == fully_charged || msg.get()->state == exploring)
        advanced_computations_bool = true;
    else
        advanced_computations_bool = false;
    
    if(msg.get()->state == in_queue)
        idle_mode = true;
    else
        idle_mode = false;
        
}

void battery_simulate::compute()
{
    /* Compute the number of elapsed seconds since the last time that we updated the battery state (since we use
     * powers) */
    ros::Duration time_diff = time_manager->simulationTimeNow() - time_last;
    double time_diff_sec = time_diff.toSec();
    elapsed_time = time_diff_sec; //TODO for debugging, should be removed...
    
    /* If there is no time difference to last computation, there is nothing to do */
    if (time_diff_sec <= 0)
        return;
    
    // If the robot is in fully
    if(do_not_consume_battery) {
        time_last = time_manager->simulationTimeNow();
        return;
    }   

    /* If the robot is charging, increase remaining battery life, otherwise compute consumed energy and decrease remaining battery life */
    if (state.charging)
    {
        remaining_energy += power_charging * time_diff_sec;
        state.soc = remaining_energy / total_energy;

        /* Check if the battery is now fully charged; notice that SOC could be higher than 100% due to how we increment
         * the remaing_energy during the charging process */
        if (state.soc >= 1)
        {
            ROS_INFO("Recharging completed");
            
            // Set battery state to its maximum values
            state.soc = 1; // since SOC cannot be higher than 100% in real life, force it to be 100%
            state.charging = false;
            state.remaining_time_charge = 0;
            remaining_energy = total_energy;
            state.remaining_time_run = maximum_running_time;
            state.remaining_distance = maximum_running_time * speed_avg;
            
            // Inform nodes that charging has been completed
            std_msgs::Empty msg; //TODO(minor) use remaining_time_charge instead of the publisher
            pub_charging_completed.publish(msg);
        }
        else {
            ROS_DEBUG("Recharging...");
            state.remaining_time_charge = (total_energy - remaining_energy) / power_charging ;
            state.remaining_time_run = remaining_energy / (power_moving * max_speed_linear + power_standing + power_basic_computations + power_advanced_computation);  
            state.remaining_distance = state.remaining_time_run * speed_avg; 
            }
    }
    else if(idle_mode) {
        remaining_energy -= power_idle * time_diff_sec; // J
        
        state.soc = remaining_energy / total_energy;
        
        //state.remaining_time_run = state.soc * total_energy; // NO!!!
        //state.remaining_time_run = mass * speed_avg / total_energy; // in s, since J = kg*m/s^2

        state.remaining_time_charge = (total_energy - remaining_energy) / power_charging ;
        state.remaining_time_run = remaining_energy / (power_moving * max_speed_linear + power_standing + power_basic_computations + power_advanced_computation);  
        state.remaining_distance = state.remaining_time_run * speed_avg;
        
    } else {
    
        /* If the robot is moving, than we have to consider also the energy consumed for moving, otherwise it is
         * sufficient to consider the fixed power cost.
         * Notice that this is clearly an approximation, since we are not sure that the robot was moving for the whole
         * interval of time: moreover, since we do not know the exact speed profile during this interval of time, we
         * overestimate the consumed energy by assuming that the robot moved at the maximum speed for the whole period.
         */
        int mult; //TODO check better
        if(advanced_computations_bool)
            mult = 1;
        else
            mult = 0;
        //if (speed_linear > 0 || speed_angular > 0) //in the foruma speed_angular is not used, so...
        if (speed_linear > 0) //TODO we should check also the robot state (e.g.: if the robot is in 'exploring', speed_linear should be zero...)
            remaining_energy -= (power_moving * max_speed_linear + power_standing + power_basic_computations + power_advanced_computation * mult) * time_diff_sec; // J
        else
            remaining_energy -= (                                  power_standing + power_basic_computations + power_advanced_computation * mult) * time_diff_sec; // J

        //ROS_ERROR("%f", remaining_energy);
        /* Update battery state */
        state.soc = remaining_energy / total_energy;
        
        //state.remaining_time_run = state.soc * total_energy; // NO!!!
        //state.remaining_time_run = mass * speed_avg / total_energy; // in s, since J = kg*m/s^2

        state.remaining_time_charge = (total_energy - remaining_energy) / power_charging ;
        state.remaining_time_run = remaining_energy / (power_moving * max_speed_linear + power_standing + power_basic_computations + power_advanced_computation);  
        state.remaining_distance = state.remaining_time_run * speed_avg; 

    }

    //ROS_ERROR("Battery: %.0f%%  ---  remaining distance: %.2fm", state.soc * 100, state.remaining_distance);
    
    /* Store the time at which this battery state update has been perfomed, so that next time we can compute againg the elapsed time interval */
    time_last = time_manager->simulationTimeNow();
}

void battery_simulate::log()
{
    std::string state_std;
    if(idle_mode)
        state_std = "idle";
    else if(state.charging) 
        state_std = "charging";
    else if(advanced_computations_bool)
        state_std = "computing";
    else if(speed_linear > 0 || speed_angular > 0)
        state_std = "moving";
    else
        state_std = "standing";
        
    ros::Duration sim_time = time_manager->simulationTimeNow() - sim_time_start;
    ros::WallDuration wall_time = ros::WallTime::now() - wall_time_start;
    
    battery_state_fs.open(battery_state_filename.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
    battery_state_fs << sim_time.toSec() << "," << wall_time.toSec() << "," << state.remaining_time_run << "," << state.remaining_time_charge << "," << state.remaining_distance << "," << state_std << std::endl;
    battery_state_fs.close();
}

void battery_simulate::publish()
{
    pub_battery.publish(state);
    //ROS_ERROR("%.1f", state.soc);
}

void battery_simulate::cb_cmd_vel(const geometry_msgs::Twist &msg)
{
    ROS_DEBUG("Received speed");
    speed_linear = msg.linear.x;
    speed_angular = msg.angular.z;
}

void battery_simulate::cb_speed(const explorer::Speed &msg)  // unused for the moment, but maybe with a better estimate
                                                             // of the battery curve...
{
    // if the average speed is very low, there is probably something wrong, set it to the value from the config file
    if (msg.avg_speed > speed_avg_init)
    {
        speed_avg = msg.avg_speed;
    }
    else
    {
        speed_avg = speed_avg_init;
    }
}

// TODO(minor) - DO WE NEED THIS???
void battery_simulate::cb_soc(const std_msgs::Float32::ConstPtr &msg)
{
    // ROS_INFO("Received SOC!!!");
}

// TODO(minor) do we need this?
//void battery_simulate::totalTime(const std_msgs::Float32::ConstPtr &msg)
//{
//    total_time = ("%F", msg->data);
//}

void battery_simulate::run() {
    while(ros::ok()) {
        ros::Duration(1).sleep(); //TODO(minor) rates???
        ros::spinOnce();
        
        // compute new battery state
        compute();

        // log battery
        log();

        // publish battery state
        publish();
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

    fs_info.open(info_file.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
    fs_info << "#power_moving,power_standing,power_charging,charge_max,power_idle,power_basic_computations,power_advanced_computationsmax_linear_speed,initial_speed_avg" << std::endl;
    fs_info << power_moving << "," << power_standing << "," << power_charging << "," << charge_max << "," << power_idle << "," << power_basic_computations << "," << power_advanced_computation << "," << max_speed_linear << "," << speed_avg_init << std::endl;
    fs_info.close();
    
    battery_state_fs.open(battery_state_filename.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
    battery_state_fs << "#sim_time,wall_time,remaining_time_run,remaining_time_charge,remaining_distance,state" << std::endl;
    battery_state_fs.close();
    
    sim_time_start = ros::Time::now();
    wall_time_start = ros::WallTime::now();
}


/*************************
 ** Debugging functions **
 *************************
 */
void battery_simulate::set_last_time() {
    time_last = time_manager->simulationTimeNow();
}

double battery_simulate::last_time_secs() {
    return time_last.toSec();
}

double battery_simulate::getChargeMax() {
    return charge_max;
}

double battery_simulate::getRemainingEnergy() {
    return remaining_energy;
}

double battery_simulate::getElapsedTime() {
    return elapsed_time;
}

void battery_simulate::spinOnce() {
    ros::spinOnce();
}
