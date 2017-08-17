#include "utilities/data_logger.h"

DataLogger::DataLogger(std::string node_name, std::string robot_name, std::string log_path) {
    std::string complete_path = createCompletePath(node_name, robot_name, log_path);
    createDirectoryFromPathIfNotExists(complete_path);
}

bool DataLogger::createLogFile(std::string filename, std::stringstream &header) { //TODO raise exception
    return false;
}

bool DataLogger::updateLogFile(std::string filename, std::stringstream &new_sample) {
    return false;   
}

std::string DataLogger::createCompletePath(std::string node_name, std::string robot_name, std::string log_path) {
    return log_path.append(node_name).append(robot_name);
}

void DataLogger::createDirectoryFromPathIfNotExists(std::string path) {
    boost::filesystem::path boost_path(path);
    if (!boost::filesystem::exists(boost_path))
    {
        ROS_INFO("Creating directory %s", path.c_str());
        try
        {
            if (!boost::filesystem::create_directories(boost_path))
            {
                ROS_ERROR("Cannot create directory %s: aborting node...", path.c_str());
                exit(-1);
            }
        }
        catch (const boost::filesystem::filesystem_error &e)
        {
            ROS_ERROR("Cannot create path %saborting node...", path.c_str());
            exit(-1);
        }
    }
    else
        ROS_INFO("Directory %s already exists: log files will be saved there", path.c_str());
}
