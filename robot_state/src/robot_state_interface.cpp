#include "robot_state/robot_state_management.h"

RobotStateApi::RobotStateApi() {//TODO RobotStatePackageApi and .cpp file name
    ROS_INFO("Creating instance of RobotStateApi");
    createServiceClients();
    fillMaps();
    ROS_INFO("Instance correctly created");
}

void RobotStateApi::createServiceClients() {
    ros::NodeHandle nh;
    set_robot_state_sc = nh.serviceClient<robot_state::SetRobotState>("robot_state/set_robot_state");
    get_robot_state_sc = nh.serviceClient<robot_state::GetRobotState>("robot_state/get_robot_state");
    try_to_lock_robot_state_ss = nh.serviceClient<robot_state::SetRobotState>("robot_state/try_to_lock_robot_state");
    unlock_robot_state_ss = nh.serviceClient<robot_state::SetRobotState>("robot_state/unlock_robot_state");
}

void RobotStateApi::fillMaps() { //TODO complete stateMap
    addStateToMaps(robot_state::INITIALIZING, new InitializingState());
    addStateToMaps(robot_state::CHOOSING_ACTION, new ChoosingActionState());
}

void RobotStateApi::addStateToMaps(unsigned int enum_val, RobotState *state) {
    enumToStateMap.insert({enum_val, state});
    stateToEnumMap.insert({state, enum_val});
}

RobotState *RobotStateApi::getRobotState() { //TODO raise exception //TODO put locks
    ROS_ERROR("calling");  
    robot_state::GetRobotState get_srv_msg;
    bool call_succeeded = get_robot_state_sc.call(get_srv_msg);
    while(!call_succeeded) { //TODO max number of retries
        ROS_ERROR("call failed");   
        call_succeeded = get_robot_state_sc.call(get_srv_msg);
    }
    return enumToStateMap.at(get_srv_msg.response.robot_state); //TODO safety check to avoid out_of_range
}

bool RobotStateApi::getRobotState2() { //TODO raise exception
    ROS_ERROR("calling");  
    robot_state::GetRobotState get_srv_msg;
    return get_robot_state_sc.call(get_srv_msg);
}

void RobotStateApi::setRobotState(RobotState *state) { //TODO raise exception
    robot_state::SetRobotState set_srv_msg;
    set_srv_msg.request.robot_state = stateToEnumMap.at(state);
    bool call_succeeded = set_robot_state_sc.call(set_srv_msg);
    while(!call_succeeded) { //TODO max number of retries
        ROS_ERROR("call failed");   
        call_succeeded = set_robot_state_sc.call(set_srv_msg); //TODO check if the state was set (it could fail due to a lock
    }
}
