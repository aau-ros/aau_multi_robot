#include "battery_state_updater.h"

BatteryStateUpdater::BatteryStateUpdater(explorer::battery_state *b) {
    loadParameters();
    initializeVariables();
}

void BatteryStateUpdater::loadParameters() {
    this->b = b; //TODO bas names
    ros::NodeHandle nh_tilde("~");
    nh_tilde.getParam("speed_avg_init", speed_avg_init); //TODO use config file instead of yaml file for the parameters
    nh_tilde.getParam("power_charging", power_charging);
    nh_tilde.getParam("power_per_speed", power_per_speed);
    nh_tilde.getParam("power_moving_fixed_cost", power_moving_fixed_cost);
    nh_tilde.getParam("power_sonar", power_sonar);
    nh_tilde.getParam("power_laser", power_laser);
    nh_tilde.getParam("power_microcontroller", power_microcontroller);
    nh_tilde.getParam("power_basic_computations", power_basic_computations);
    nh_tilde.getParam("power_advanced_computations", power_advanced_computations);
    nh_tilde.getParam("max_linear_speed", max_speed_linear);
    nh_tilde.getParam("maximum_traveling_distance", maximum_traveling_distance);
}

void BatteryStateUpdater::initializeVariables() {
    speed_avg = speed_avg_init;
    last_pose_x = 0, last_pose_y = 0;
    traveled_distance = 0;
    last_traveled_distance = 0;
    total_traveled_distance = 0;
}

void BatteryStateUpdater::subscribeToTopics() {
    ros::NodeHandle nh;
    avg_speed_sub = nh.subscribe("avg_speed", 10, &BatteryStateUpdater::avgSpeedCallback, this); //TODO queue lenght
    cmd_vel_sub = nh.subscribe("cmd_vel", 10, &BatteryStateUpdater::cmdVelCallback, this);
    pose_sub = nh.subscribe("amcl_pose", 10, &BatteryStateUpdater::poseCallback, this);
}

void BatteryStateUpdater::initializeBatteryState() {
    b->charging = false;
    b->soc = 1; // (adimensional) // TODO(minor) if we assume that the robot starts fully_charged
    b->remaining_time_charge = 0; // since the robot is assumed to be fully charged when the exploration starts
    b->remaining_distance = maximum_traveling_distance;
    b->remaining_time_run = maximum_traveling_distance * speed_avg_init; //s //TODO(minor) "maximum" is misleading: use "estimated"...
    b->maximum_traveling_distance = maximum_traveling_distance;
    b->fully_charged = true; //TODO assumption
    b->consumed_energy_A = 0;
    b->consumed_energy_B = 0;
}

void BatteryStateUpdater::avgSpeedCallback(const explorer::Speed &msg)
{
    // if the average speed is very low, there is probably something wrong, set it to the value from the config file
    if (msg.avg_speed > speed_avg_init)
        speed_avg = msg.avg_speed;
    else
        speed_avg = speed_avg_init;
}

void BatteryStateUpdater::poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose) {
    double pose_x = pose->pose.pose.position.x;
    double pose_y = pose->pose.pose.position.y;
    
    mutex_traveled_distance.lock();
    traveled_distance += sqrt( (last_pose_x-pose_x)*(last_pose_x-pose_x) + (last_pose_y-pose_y)*(last_pose_y-pose_y) ); //TODO use hypot
    mutex_traveled_distance.unlock();
    
    last_pose_x = pose_x;
    last_pose_y = pose_y;
}

void BatteryStateUpdater::cmdVelCallback(const geometry_msgs::Twist &msg)
{
    ROS_DEBUG("Received speed");
    speed_linear = msg.linear.x;
    speed_angular = msg.angular.z;
}

void BatteryStateUpdater::handle(InitializingState *r) {
    substractEnergyRequiredForSensing();
    substractEnergyRequiredForBasicComputations();
}

void BatteryStateUpdater::handle(ChoosingActionState *r) {
    substractEnergyRequiredForSensing();
    substractEnergyRequiredForBasicComputations();
}

//void battery_simulate::cb_robot(const adhoc_communication::EmRobot::ConstPtr &msg)
//{
//    if(initializing && msg.get()->state == moving_to_frontier) {
//        if(counter_moving_to_frontier == 0)
//            counter_moving_to_frontier++;
//        else {
//            initializing = false;
//            ROS_INFO("Finished initialization procedure");
//        }
//        return;

//        //TODO use this code
//        //if(initializing) {
//        //    initializing = false;
//         //   ROS_INFO("Finished initialization procedure");
//        //}
//        //return;  
//    } 
//    
//    if(initializing)
//        return;

//    if (msg.get()->state == charging)
//    {
//        ROS_DEBUG("Start recharging");
//        state.charging = true;
//        prev_consumed_energy_A = consumed_energy_A;
//    }
//    else {
//        /* The robot is not charging; if the battery was previously under charging, it means that the robot aborted the
//        recharging process */
//        if (state.charging == true)
//        {
//            ROS_DEBUG("Recharging aborted");
//            state.charging = false;
//            state.fully_charged = false; //TODO reduntant?
//        }
//    }
//    
//    if(msg.get()->state == fully_charged) //when the robot is fully_charged, it is left a little bit at the DS to allow him to compute the next DS without consuming energy, so that the check of the reachability of frontiers also using the Ds graph makes sense
//        do_not_consume_battery = true;
//    else
//        do_not_consume_battery = false;
//    
////    if(msg.get()->state == fully_charged || msg.get()->state == exploring || msg.get()->state == exploring_for_graph_navigation)
//    if(msg.get()->state == exploring || msg.get()->state == exploring_for_graph_navigation)
//        advanced_computations_bool = true;
//    else
//        advanced_computations_bool = false;
//    
//    if(msg.get()->state == in_queue)
//        idle_mode = true;
//    else
//        idle_mode = false;
//        
//}

//void battery_simulate::compute()
//{
//    BatteryStateUpdater c(&state);
//    (robot_state_manager.getRobotState())->accept(&c);
//    
//    //robot_state::robot_state_t robot_state = static_cast<robot_state::robot_state_t>(get_srv_msg.response.robot_state); //TODO use visitor instead of switch 
//    robot_state::robot_state_t robot_state = static_cast<robot_state::robot_state_t>(1);
//        

//    /* Compute the number of elapsed seconds since the last time that we updated the battery state (since we use
//     * powers) */
//    ros::Duration time_diff = time_manager->simulationTimeNow() - time_last;
//    double time_diff_sec = time_diff.toSec();
//    elapsed_time = time_diff_sec; //TODO for debugging, should be removed...

//    
//    
//    /* If there is no time difference to last computation, there is nothing to do */
//    if (time_diff_sec <= 0)
//        return;

//    state.fully_charged = false;
//    /* If the robot is charging, increase remaining battery life, otherwise compute consumed energy and decrease remaining battery life */
//    if (robot_state == robot_state::CHARGING)
//    {
//        rechargeBattery(time_diff_sec);
//        
//        /* Check if the battery is now fully charged; notice that SOC could be higher than 100% due to how we increment
//         * the remaing_energy during the charging process */
////        if (state.soc >= 1)
//        if(consumed_energy_A <=0 && consumed_energy_B <= 0)
//        {
//            ROS_INFO("Recharging completed");
//            
//            state.charging = false;
//            state.fully_charged = true;
//             
//            consumed_energy_A = 0;
//            consumed_energy_B = 0;
//            
//            // Set battery state to its maximum values 
//            state.remaining_distance = maximum_traveling_distance;
//            state.remaining_time_charge = 0;
//            state.soc = 1; // since SOC cannot be higher than 100% in real life, force it to be 100%
//            
//            robot_state::SetRobotState set_srv_msmg;
//            bool call_succeeded = set_robot_state_sc.call(set_srv_msmg);
//            while(!call_succeeded) { //TODO duplicated code... use a function
//                call_succeeded = set_robot_state_sc.call(set_srv_msmg);
//                ROS_ERROR("..");   
//            }
//            
//        }
//    }
//    else if (do_not_consume_battery) {
//        time_last = time_manager->simulationTimeNow();
//        return;
//    } 
//    else if(robot_state == robot_state::IN_QUEUE) {
//        consumed_energy_B += power_idle * time_diff_sec; // J
//        
//    }
//    else if(isRobotMoving()) {
//    
//        /* If the robot is moving, than we have to consider also the energy consumed for moving, otherwise it is
//         * sufficient to consider the fixed power cost.
//         * Notice that this is clearly an approximation, since we are not sure that the robot was moving for the whole
//         * interval of time: moreover, since we do not know the exact speed profile during this interval of time, we
//         * overestimate the consumed energy by assuming that the robot moved at the maximum speed for the whole period.
//         */
//        
//        if (speed_linear > 0) { //TODO we should check also the robot state (e.g.: if the robot is in 'exploring', speed_linear should be zero...); notice that 
//            consumed_energy_A += (power_moving_fixed_cost + power_per_speed * speed_linear) * time_diff_sec; // J
//        }
//        
//        consumed_energy_B += (power_sonar + power_laser + power_microcontroller + power_basic_computations + power_advanced_computations) * time_diff_sec; // J
//        
//        /* Update battery state */
//        state.remaining_time_charge = (consumed_energy_A + consumed_energy_B) / power_charging ;
//        
//        mutex_traveled_distance.lock();
//        state.remaining_distance -= traveled_distance;
////        ROS_ERROR("%.2f", traveled_distance);

//        last_traveled_distance = traveled_distance;
//        total_traveled_distance += traveled_distance;

//        traveled_distance = 0;
//        mutex_traveled_distance.unlock();
//        
//    }
//    
//    state.remaining_time_run = state.remaining_distance * speed_avg;
//    state.soc = state.remaining_distance / maximum_traveling_distance;

////    ROS_ERROR("SOC: %.0f%%; remaining distance: %.2fm", state.soc * 100, state.remaining_distance);
//    
//    /* Store the time at which this battery state update has been perfomed, so that next time we can compute againg the elapsed time interval */
//    time_last = time_manager->simulationTimeNow();
//}

//void battery_simulate::rechargeBattery(double time_diff_sec) {
//    ROS_INFO("Recharging battery");
//    
//    
//    double ratio_A = -1, ratio_B = -1;
//    if(consumed_energy_A < 0 && consumed_energy_B < 0) {
//        ROS_FATAL("this should not happen...");
//        return;
//    }
//    else if(consumed_energy_A <= 0) {
//        ratio_A = 0.0;
//        ratio_B = 1.0;
//        consumed_energy_A = 0;
//        ROS_ERROR("this should not happen..."); //FIXME actually this could happen since B is also consumed while charging...
//    }   
//    else if(consumed_energy_B <= 0) {
//        ratio_A = 1.0;
//        ratio_B = 0.0;
//        consumed_energy_B = 0;
//        ROS_ERROR("this should not happen...");
//    }
//    else {
//        ratio_A = consumed_energy_A / (consumed_energy_A + consumed_energy_B);
//        ratio_B = consumed_energy_B / (consumed_energy_A + consumed_energy_B);
//    }

//    if(ratio_A < 0 || ratio_A > 1 || ratio_B < 0 || ratio_B > 1 || fabs(ratio_A + ratio_B - 1.0) > 0.01 ) {
//        ROS_FATAL("strange ratio");
//        return;
//    }

//    consumed_energy_A -= ratio_A * power_charging * time_diff_sec;
//    consumed_energy_B -= ratio_B * power_charging * time_diff_sec;
//    consumed_energy_B += power_idle * time_diff_sec;

//    state.remaining_time_charge = (consumed_energy_A + consumed_energy_B) / power_charging ;
//    state.remaining_distance = (prev_consumed_energy_A - consumed_energy_A) / prev_consumed_energy_A * maximum_traveling_distance;
//    if(state.remaining_distance > maximum_traveling_distance)
//        state.remaining_distance = maximum_traveling_distance;
//}
