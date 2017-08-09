#include "time_manager.h"

TimeManager::TimeManager() {

}

ros::Time TimeManager::simulationTimeNow() { //TODO is it possible to have something like SimulationTime::now() ?
    return ros::Time::now();
}
