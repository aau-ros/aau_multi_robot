#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include <string>
#include <sstream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <ros/ros.h>

class DataLogger {
public:
    DataLogger(const std::string &node_name, const std::string &robot_name, const std::string &log_path);
    void createLogFile(const std::string &filename, const std::string &header);
    void createLogFile(const std::string &filename, std::stringstream &header);
    void updateLogFile(const std::string &filename, const std::string &new_sample);
    void updateLogFile(const std::string &filename, std::stringstream &new_sample);

private:
    std::string complete_dir_path;

    std::string composeCompletePath(std::string node_name, std::string robot_name, std::string log_path);
    void createDirectoryFromPathIfNotExists(std::string path);
};

#endif // DATA_LOGGER_H
