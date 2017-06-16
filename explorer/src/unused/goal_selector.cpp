#include "goal_selector.h"

GoalSelector::GoalSelector() {
    ros::NodeHandle nh_tilde("~");
    nh_tilde.param("cost_function_selection", cost_function_selection, 0);
    nh_tilde.param("w1", w1, 0);
    nh_tilde.param("w2", w2, 0);
    nh_tilde.param("w3", w3, 0);
    nh_tilde.param("w4", w4, 0);
}

int GoalSelector::selectFrontierAsGoal() {
    return 0;
}
