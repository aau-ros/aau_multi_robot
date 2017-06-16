int distance_temp = exploration->trajectory_plan(target_ds_x - 2,
target_ds_y - 2);
int x_temp = target_ds_x - 2;
int y_temp = target_ds_y - 2;
if(distance_temp > exploration->trajectory_plan(target_ds_x - 2,
target_ds_y + 2)) {
    distance_temp = exploration->trajectory_plan(target_ds_x - 2,
target_ds_y + 2);
    x_temp = target_ds_x - 2;
    y_temp = target_ds_y + 2;
} else if (distance_temp > exploration->trajectory_plan(target_ds_x +
2, target_ds_y + 2)) {
    distance_temp = exploration->trajectory_plan(target_ds_x + 2,
target_ds_y + 2);
    x_temp = target_ds_x + 2;
    y_temp = target_ds_y + 2;
} else if (distance_temp > exploration->trajectory_plan(target_ds_x +
2, target_ds_y - 2)) {
    distance_temp = exploration->trajectory_plan(target_ds_x + 2,
target_ds_y - 2);
    x_temp = target_ds_x + 2;
    y_temp = target_ds_y - 2;
}



ROS_ERROR("%d", exploration->trajectory_plan(target_ds_x - 2,
target_ds_y - 2));
ROS_ERROR("%d", exploration->trajectory_plan(target_ds_x - 2,
target_ds_y    ));
ROS_ERROR("%d", exploration->trajectory_plan(target_ds_x - 2,
target_ds_y - 2));
ROS_ERROR("%d", exploration->trajectory_plan(target_ds_x    ,
target_ds_y - 2));
ROS_ERROR("%d", exploration->trajectory_plan(target_ds_x    ,
target_ds_y    ));
ROS_ERROR("%d", exploration->trajectory_plan(target_ds_x    ,
target_ds_y + 2));
ROS_ERROR("%d", exploration->trajectory_plan(target_ds_x + 2,
target_ds_y - 2));
ROS_ERROR("%d", exploration->trajectory_plan(target_ds_x + 2,
target_ds_y    ));
ROS_ERROR("%d", exploration->trajectory_plan(target_ds_x + 2,
target_ds_y + 2));
                

void going_charging_callback(const std_msgs::Empty::ConstPtr &msg)
{
ROS_ERROR("\n\t\e[1;34m going_charging_callback \e[0m");
robot_state_next = going_charging_next;
}

void going_queue_callback(const std_msgs::Empty::ConstPtr &msg)
{
ROS_ERROR("\n\t\e[1;34m going_queue_callback \e[0m");
robot_state_next = going_queue_next;
}

void exploring_callback(const std_msgs::Empty::ConstPtr &msg)
{
ROS_ERROR("\n\t\e[1;34m expl_callback \e[0m");
robot_state_next = exploring_next;
}

void fully_charged_callback(const std_msgs::Empty::ConstPtr &msg)
{
ROS_ERROR("\n\t\e[1;34m fully_callback \e[0m");
robot_state_next = fully_charged_next;
}


adhoc_communication::EmRobot msg;
if(new_state == in_queue) {
    msg.state = in_queue;
    robot_state = in_queue;
    ROS_ERROR("\e[1;34m in_queue \e[0m");
} else if(new_state == going_charging) {
    msg.state = going_charging;
    robot_state = going_charging;
    ROS_ERROR("\e[1;34m going_charging \e[0m");
} else if(new_state == charging) {
    msg.state = charging;
    robot_state = charging;
    ROS_ERROR("\e[1;34m charging \e[0m");
} else if(new_state == moving_to_frontier) {
    robot_state = moving_to_frontier;
    msg.state = moving_to_frontier;
    ROS_ERROR("\e[1;34m moving_to_frontier \e[0m");
} else if(new_state == exploring) {
    robot_state = exploring;
    msg.state = exploring;
    ROS_ERROR("\e[1;34m exploring \e[0m");
} else if(new_state == checking_vacancy) {
    robot_state = checking_vacancy;
    msg.state = checking_vacancy; 
    ROS_ERROR("\e[1;34m checking_vacancy \e[0m");
} else if(new_state == auctioning) {
    robot_state = auctioning;
    msg.state = auctioning;
    ROS_ERROR("\e[1;34m auctioning \e[0m");
} else if(new_state == finished) {
    robot_state = finished;
    msg.state = finished;
    ROS_ERROR("\e[1;34m finished \e[0m");
} else if(new_state == fully_charged) {
    robot_state = fully_charged;
    msg.state = fully_charged;
    ROS_ERROR("\e[1;34m fully_charged \e[0m");
} else if(new_state == going_in_queue) {
    robot_state = going_in_queue;
    msg.state = going_in_queue;
    ROS_ERROR("\e[1;34m going_in_queue \e[0m");
} else if(new_state == going_checking_vacancy) {
    robot_state = going_checking_vacancy;
    msg.state = going_checking_vacancy;
    ROS_ERROR("\e[1;34m going_checking_vacancy \e[0m");
} else
    ROS_ERROR("The next robot state is not going to be properly handled");
pub_robot.publish(msg);
