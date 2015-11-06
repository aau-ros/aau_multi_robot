#include <ros/ros.h>
#include <ros/console.h>
#include <battery_mgmt.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "main");
    battery_mgmt bm;
    ros::Rate loop_rate(0.5); // Hz

    while(ros::ok()){

        // increase battery charge (don't count time)
        if(charging){
            ROS_ERROR("charging");

            ROS_ERROR("charging");
            charge_time -= (1/rate);
            if(charge_time <= 0){
                home = false;
                ROS_ERROR("stop charging");
            }
            ROS_INFO("Charging time: %f ",charge_time);
            if(charge_time < 0)
                charge_time = 0;
            charge_msg.remaining_time = charge_time;

            actual_charge = (1 - remaining_charge_time/full_charge_time/3600) * battery_charge;
        }

        // decrease battery charge and compute standing and moving times
        else{
            ROS_ERROR("discharging");
            // robot is standing
            if(x_linear == 0 && z_angular == 0){
                actual_charge -= standing_consumption / (rate * 3600 );
                standing_time += 1/rate;
            }
            // robot is moving
            else{
                actual_charge -= moving_consumption / (rate * 3600 );
                moving_time += 1/rate;
            }

            // calculate percentages of standing and moving times
            moving_perc = moving_time / (moving_time + standing_time);
            standing_perc = standing_time / (standing_time + moving_time);
            if(moving_perc < 0.5){
                moving_perc = 0.5;
                standing_perc = 0.5;
            }

            ROS_ERROR("discharging");
            charge_time = full_charge_time * (100-battery)/100 * 60*60;
            charge_msg.remaining_time = charge_time;
        }

        // percentage of battery charge remaining
        battery_state.percent = (actual_charge * 100) / battery_charge;
        if(battery_state.percent < 0)
            battery_state.percent = 0;
        if(battery_state.percent > 100)
            battery_state.percent = 100;

        // remaining distance the robot can still travel in meters
        battery_state.remaining_distance = avg_speed * actual_charge/moving_consumption*3600;

        // remaining time the robot can still work in seconds
        battery_state.remaining_time = actual_charge*3600 * (moving_perc/moving_consumption + standing_perc/standing_consumption);

        int bs = battery_state.percent;
        if((bs % 10) == 0)
        {
            if (debugShown == false)
            {
                ROS_ERROR("Battery: %d%%  ---  remaining distance: %.2fm", bs, battery_state.remaining_distance);
                debugShown = true;
            }
        }
        else
            debugShown = false;

        battery_pub.publish(battery_state);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
