#ifndef UPDATEMANAGER_H
#define UPDATEMANAGER_H

#include "iostream"
#include "vector"
#include "ros/console.h"
class updateManager
{
private:
    std::vector < std::vector < int > *>* updateInformation;
public:
    updateManager();
    void addNewUpdateList();
    void addToupdateList(int indexOfMap,std::vector < int > values);
    std::vector<int>* getMissingUpdateOfRobot(int indexOfMap);
    std::vector<int>* getUpdateListOfrobot(int indexOfMap);
    bool isUpdatesMissing(int indexOfMap);
    int getLatestUpdateVersionOfRobot(int indexOfMap);
};

#endif // UPDATEMANAGER_H
