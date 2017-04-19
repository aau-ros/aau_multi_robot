#include <battery_simulate.h>

using namespace std;

battery_simulate::battery_simulate()
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
  //  state.remaining_time_run = charge_max * (perc_moving / power_moving + perc_standing / power_standing) * 3600;
 //   state.remaining_distance = speed_avg_init * charge_max / power_moving * 3600;


    bool debugShown = false;

    // advertise topics
    pub_battery = nh.advertise<energy_mgmt::battery_state>("battery_state", 1);

    // subscribe to topics
    sub_charge = nh.subscribe("going_to_recharge", 1, &battery::cb_charge, this);
    sub_speed = nh.subscribe("avg_speed", 1, &battery::cb_speed, this);

    //TODO: do we need this?
    sub_time = nh.subscribe("totalTime", 1, &battery::totalTime, this);

}

void battery_simulate::compute()
{
    // calculate time difference
    ros::Duration time_diff = ros::Time::now() - time_last;
    time_last = ros::Time::now();
    double time_diff_sec = time_diff.toSec();
    // if there is no time difference to last computation then there is nothing to do
    if(time_diff_sec <= 0)
        return;

    //TODO: move calculations to pioneer_battery node or merge pioneer node into this one
    state.remaining_time_run = (total_time * state.soc) / (100 * ((0.7551 * speed_linear) + 0.3959));
    state.remaining_distance = state.remaining_time_run * speed_linear;
//     if you don't need the max. time left and the max. remaining distance, add 1 to the variable j of array[][j] in
//     battery_measure.cpp, line 87 row 46 before publishing stateOfCharge (stoch.data)

}

void battery_simulate::output()
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

void battery_simulate::publish()
{
    pub_battery.publish(state);
}

void battery_simulate::cb_charge(const std_msgs::Empty::ConstPtr &msg)
{
    ROS_ERROR("Starting to recharge");
    state.charging = true;
}

void battery_simulate::cb_cmd_vel(const geometry_msgs::Twist &msg)
{
    speed_linear = msg.linear.x;
    speed_angular = msg.angular.z;
}

void battery_simulate::cb_speed(const explorer::Speed &msg)
{

    // if the average speed is very low, there is probably something wrong, set it to the value from the config file
    if(msg.avg_speed > speed_avg_init){
        speed_avg = msg.avg_speed;
    }
    else{
        speed_avg = speed_avg_init;
    }
}

void battery_simulate::cb_soc(const std_msgs::Float32::ConstPtr& msg)
{
    state.soc = ("%F", msg->data);
}

//TODO: do we need this?
void battery_simulate::totalTime(const std_msgs::Float32::ConstPtr& msg)
{
    total_time = ("%F", msg->data);
}


