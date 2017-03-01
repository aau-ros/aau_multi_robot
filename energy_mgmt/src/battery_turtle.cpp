

// todo: subscribe to diagnostics topic
//sub3 = nh.subscribe("diagnostics_agg",1000,&Explorer::turtlebot_callback,this);


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