#include "mock_time_manager.h"

MockTimeManager::MockTimeManager() {
    index = 0;
}

ros::Time MockTimeManager::simulationTimeNow() {
    double next_time = -1; 
    if(index < time_list.size()) {
        next_time = time_list.at(index);
        index++;
        return ros::Time(next_time);
    }
    else
        return ros::Time(-1); //error
}

void MockTimeManager::addTime(double time) {
    time_list.push_back(time);
}
