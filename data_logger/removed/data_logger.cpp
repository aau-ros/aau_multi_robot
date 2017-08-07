#include "data_logger.h"

DataLogger::DataLogger() {
    createServices();
}

void DataLogger::createServices() {
    ros::NodeHandle nh;
    create_log_file_ss = nh.advertiseService("data_logger/create_log_file", &DataLogger::createLogFileCallback, this);
    update_log_file_ss = nh.advertiseService("data_logger/update_log_file", &DataLogger::updateLogFileCallback, this);
}

bool DataLogger::createLogFileCallback(data_logger::CreateLogFile::Request &req, data_logger::CreateLogFile::Response &res) {
    
}

bool DataLogger::updateLogFileCallback(data_logger::UpdateLogFile::Request &req, data_logger::UpdateLogFile::Response &res) {

}
