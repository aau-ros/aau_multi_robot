#include "computer.h"

Computer2::Computer2(explorer::battery_state *b) {
    this->b = b; //TODO bas names
    ros::NodeHandle nh_tilde("~");
    nh_tilde.getParam("speed_avg", speed_avg_init); //TODO speed_avg maybe is not a good name
    nh_tilde.getParam("power_charging", power_charging); // W (i.e, watt)
    nh_tilde.getParam("power_per_speed", power_per_speed);     // W/(m/s)
    nh_tilde.getParam("power_moving_fixed_cost", power_moving_fixed_cost);     // W/(m/s)
    nh_tilde.getParam("power_sonar", power_sonar); // W
    nh_tilde.getParam("power_laser", power_laser); // W
    nh_tilde.getParam("power_microcontroller", power_microcontroller); // W
    nh_tilde.getParam("power_basic_computations", power_basic_computations);  // W
    nh_tilde.getParam("power_advanced_computations", power_advanced_computations);  // W
    nh_tilde.getParam("max_linear_speed", max_speed_linear); // m/s
    nh_tilde.getParam("maximum_traveling_distance", maximum_traveling_distance); // m/s
};
