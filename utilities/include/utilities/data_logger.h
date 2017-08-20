#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include <string>
#include <sstream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <ros/ros.h>

class DataLogger {
public:
    DataLogger(std::string node_name, std::string robot_name, std::string log_path);
    void createLogFile(std::string filename, std::stringstream *header);
    void updateLogFile(std::string filename, std::stringstream *new_sample);

private:
    std::string complete_dir_path;

    std::string createCompletePath(std::string node_name, std::string robot_name, std::string log_path);
    void createDirectoryFromPathIfNotExists(std::string path);
};

#endif // DATA_LOGGER_H
