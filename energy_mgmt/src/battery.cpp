#include <battery.h>

using namespace std;

battery::battery()
{
    // read parameters
    nh.getParam("energy_mgmt/speed_avg", speed_avg_init);
    nh.getParam("energy_mgmt/power_charging", power_charging);
    nh.getParam("energy_mgmt/power_moving", power_moving);
    nh.getParam("energy_mgmt/power_standing", power_standing);
    nh.getParam("energy_mgmt/charge_max", charge_max);


    // initialize private variables
    charge = charge_max;
    speed_linear = 0;
    speed_angular = 0;
    time_moving = 0;
    time_standing = 0;
    perc_moving = 0.5;
    perc_standing = 0.5;
    output_shown = false;
    time_last = ros::Time::now();

    // initialize message
    state.charging = false;
    state.soc = 100;
    state.remaining_time_charge = 0;
    state.remaining_time_run = charge_max * (perc_moving / power_moving + perc_standing / power_standing) * 3600;
    state.remaining_distance = speed_avg_init * charge_max / power_moving * 3600;


    bool debugShown = false;

    // get robot platform
//     std::string platform;
//     ros::get_environment_variable(platform, "ROBOT_PLATFORM");
//     ROS_INFO("Robot platform: %s",platform.c_str());
//     if(platform.compare("turtlebot") == 0){
//         // todo: subscribe to diagnostics topic
//         //sub3 = nh.subscribe("diagnostics_agg",1000,&Explorer::turtlebot_callback,this);
//     }
//     else if(platform.compare("pioneer3dx") == 0 || platform.compare("pioneer3at") == 0){
//         // todo: read voltage and convert to percentage
//     }
//     else{
//         // default: we are simulating
//     }

    // advertise topics
    pub_battery = nh.advertise<energy_mgmt::battery_state>("battery_state", 1);

    // subscribe to topics
    sub_charge = nh.subscribe("going_to_recharge", 10, &battery::cb_charge, this);
    sub_cmd_vel = nh.subscribe("cmd_vel", 1, &battery::cb_cmd_vel, this);
    sub_speed = nh.subscribe("avg_speed", 1, &battery::cb_speed, this);
}

void battery::compute()
{
    // calculate time difference
    ros::Duration time_diff = ros::Time::now() - time_last;
    time_last = ros::Time::now();
    double time_diff_sec = time_diff.toSec();
    // if there is no time difference to last computation then there is nothing to do
    if(time_diff_sec <= 0)
        return;

    // increase battery charge
    if(state.charging == true){
        charge += power_charging * time_diff_sec / 3600;

        // battery is fully charged
        if(charge >= charge_max){
            state.charging = false;
            charge = charge_max;
            ROS_ERROR("Stopped recharging");
        }
    }

    // decrease battery charge and compute standing and moving times
    else{
        // robot is standing
        if(speed_linear == 0 && speed_angular == 0){
            charge -= power_standing * time_diff_sec / 3600;
            time_standing += time_diff_sec;
        }

        // robot is moving
        else{
            charge -= power_moving * time_diff_sec / 3600;
            time_moving += time_diff_sec;
        }

        // battery is dead
        if(charge <= 0){
            charge = 0;
            ROS_ERROR("Battery depleted!");
        }

        // percentages of standing and moving times
        perc_moving = time_moving / (time_moving + time_standing);
        perc_standing = time_standing / (time_standing + time_moving);
        if(perc_moving < 0.5){
            perc_moving = 0.5;
            perc_standing = 0.5;
        }
    }

    // battery soc
    state.soc = charge / charge_max * 100;

    // remaining time until the battery is fully charged in seconds
    state.remaining_time_charge = (charge_max - charge) / power_charging * 3600;

    // remaining time the robot can still work in seconds
    state.remaining_time_run = charge * (perc_moving / power_moving + perc_standing / power_standing) * 3600;

    // remaining distance the robot can still travel in meters
    state.remaining_distance = speed_avg * charge / power_moving * 3600;
}

void battery::output()
{
    if((int(state.soc) % 10) == 0)
    {
        if (output_shown == false)
        {
            ROS_ERROR("Battery: %.0f%%  ---  remaining distance: %.2fm", state.soc, state.remaining_distance);
            output_shown = true;
        }
    }
    else
        output_shown = false;
}

void battery::publish()
{
    pub_battery.publish(state);
}

void battery::cb_charge(const std_msgs::Empty::ConstPtr &msg)
{
    ROS_ERROR("Starting to recharge");
    state.charging = true;
}

void battery::cb_cmd_vel(const geometry_msgs::Twist &msg)
{
    speed_linear = msg.linear.x;
    speed_angular = msg.angular.z;
}

void battery::cb_speed(const explorer::Speed &msg){

    // if the average speed is very low, there is probably something wrong, set it to the value from the config file
    if(msg.avg_speed > speed_avg_init){
        speed_avg = msg.avg_speed;
    }
    else{
        speed_avg = speed_avg_init;
    }
}

/**
 * Get the charging state for the turtlebot
 * This information is read from the diagnostic array which is provided by the diagnostic aggregator
 */
// void battery::turtlebot_callback(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg)
// {
//     for( size_t status_i = 0; status_i < msg->status.size(); ++status_i )
//     {
//         if( msg->status[status_i].name.compare("/Power System/Battery") == 0)
//         {
//             for (size_t value_i = 0; value_i < msg->status[status_i].values.size(); ++value_i)
//             {
//                 if( msg->status[status_i].values[value_i].key.compare("Percent") == 0 )
//                 {
//                     battery_charge = ::atof(msg->status[status_i].values[value_i].value.c_str());
//                 }
//
//                 if( msg->status[status_i].values[value_i].key.compare("Charging State") == 0 )
//                 {
//                     if(msg->status[status_i].values[value_i].value.c_str() == "Not Charging")
//                     {
//
//                     }
//                 }
//             }
//         }
//     }
// }
