#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include <ros/ros.h>
#include "data_logger/CreateLogFile.h"
#include "data_logger/UpdateLogFile.h"

class DataLogger {
public:
    DataLogger();
    
private:
    ros::ServiceServer create_log_file_ss;
    ros::ServiceServer update_log_file_ss;
    
    void createServices();
    
    bool createLogFileCallback(data_logger::CreateLogFile::Request &req, data_logger::CreateLogFile::Response &res);
    bool updateLogFileCallback(data_logger::UpdateLogFile::Request &req, data_logger::UpdateLogFile::Response &res);

};

#endif // DATA_LOGGER_H
