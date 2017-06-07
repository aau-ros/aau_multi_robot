#ifndef GOAL_SELECTOR_H
#define GOAL_SELECTOR_H

#include <ros/ros.h>
#include <ros/console.h>
#include "frontier.h"

class GoalSelector
{
    public:
        GoalSelector();
        //int selectFrontierAsGoal(std::vector<frontier_t>::iterator it);
        int selectFrontierAsGoal();
        
    private:
        int w1;
        int w2;
        int w3;
        int w4;
        int cost_function_selection;
};

#endif //GOAL_SELECTOR_H
