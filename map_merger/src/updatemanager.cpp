#include "updatemanager.h"

updateManager::updateManager()
{
    updateInformation = new std::vector< std::vector < int > *>();
}

void updateManager::addNewUpdateList()
{
    updateInformation->push_back(new std::vector<int>);
}

void updateManager::addToupdateList(int indexOfMap, std::vector < int > values)
{
    ROS_DEBUG("in addToupdateList,indexOfMap:%i,size of values:%lu",indexOfMap,values.size());
    std::vector<int>* tmp = updateInformation->at(indexOfMap);
    ROS_DEBUG("in addToupdateList size tmp:%lu",tmp->size());
    bool add = true;
    if(values.size() == 1 && tmp->size() > 1)
    {
        int i = tmp->at(tmp->size()-1)+1;
        int j =  values.at(0);
        if(i == j)
        {
             ROS_DEBUG("FAST %i == %i",i,j);
             tmp->push_back(values.at(0));
             ROS_DEBUG("adding revision number %i, to list at index:%i",values.at(0),indexOfMap);
             add = false;
        }
    }

    if(add)
    {
        {
            for(int i = 0; i < values.size(); i++)
            {
                for(int j = 0; j < tmp->size();j++)
                {
                    //check if list already contains that update number
                    if(tmp->at(j) == values.at(i))
                    {
                        add= false;
                        ROS_DEBUG("%i == %i",tmp->at(j),values.at(i));
                        break;
                    }
                    else
                    {
                        ROS_DEBUG("%i !! %i",tmp->at(j),values.at(i));
                    }

                }
                if(add)
                {
                    tmp->push_back(values.at(i));
                    ROS_DEBUG("adding revision number %i, to list at index:%i",values.at(i),indexOfMap);
                }
                add = true;
            }
        }
    }
    if(tmp->size() > 1)
    {
        //if the latest value is smaller then the value before, i need to sort my list
        if(tmp->at(tmp->size() - 1) > tmp->at(tmp->size() - 2))
        {
            std::sort(tmp->begin(),tmp->end());
        }
    }
}
std::vector<int>* updateManager::getUpdateListOfrobot(int indexOfMap)
{
   // ROS_DEBUG("Returning list of updates, size of list:%lu",updateInformation->at(indexOfMap)->size());
    return updateInformation->at(indexOfMap);
}
int updateManager::getLatestUpdateVersionOfRobot(int indexOfMap)
{
    return getUpdateListOfrobot(indexOfMap)->at(getUpdateListOfrobot(indexOfMap)->size()-1);
}

std::vector<int>* updateManager::getMissingUpdateOfRobot(int indexOfMap)
{
    std::vector<int>* missingUpdates = new std::vector<int>();
    int indexInCurrentList = 0;
    std::vector<int>* tmp =  getUpdateListOfrobot(indexOfMap);

    for(int i = 0; i < getLatestUpdateVersionOfRobot(indexOfMap);i++)
    {
        if(tmp->at(indexInCurrentList) == i)
        {
            indexInCurrentList++;
        }
        else
        {
            missingUpdates->push_back(i);
        }
    }
    return missingUpdates;
}

bool updateManager::isUpdatesMissing(int indexOfMap)
{
    if(getUpdateListOfrobot(indexOfMap)->size() == getLatestUpdateVersionOfRobot(indexOfMap) +1)
        return false;
    else return true;
}
