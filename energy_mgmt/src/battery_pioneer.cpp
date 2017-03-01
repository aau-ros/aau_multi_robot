
        sub_cmd_vel = nh.subscribe("/RosAria/cmd_vel", 1, &battery::cb_cmd_vel, this);
        sub_soc = nh.subscribe("stateOfCharge", 1, &battery::cb_soc, this);


void battery::compute()
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