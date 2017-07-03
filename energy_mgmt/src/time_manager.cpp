#include "time_manager.h"

TimeManager::TimeManager() {

}

ros::Time TimeManager::simulationTimeNow() {
    return ros::Time::now();
}
