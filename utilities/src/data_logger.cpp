#include "utilities/data_logger.h"

//TODO a lot of duplicated code
//TODO maybe we could use RAII
DataLogger::DataLogger(const std::string &node_name, const std::string &robot_name, const std::string &log_path) {
    complete_dir_path = composeCompletePath(node_name, robot_name, log_path);
    createDirectoryFromPathIfNotExists(complete_dir_path);
}

std::string DataLogger::composeCompletePath(std::string node_name, std::string robot_name, std::string log_path) {
    return log_path.append("/" + node_name).append("/" + robot_name);
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
            ROS_ERROR("Cannot create path %s: aborting node...", path.c_str());
            exit(-1);
        }
    }
    else
        ROS_INFO("Directory %s already exists: log files will be saved there", path.c_str());
}

void DataLogger::createLogFile(const std::string &filename, const std::string &header) { //TODO raise exception
    complete_file_path = complete_dir_path.append("/" + filename);
    fs.open(complete_file_path.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
    fs << header;
    fs.close();
}

//void DataLogger::createLogFile(std::string filename, std::string &header) { //TODO raise exception
//    std::string complete_file_path = complete_dir_path.append("/" + filename);
//    std::fstream fs;
//    fs.open(complete_file_path.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
//    fs << header;
//    fs.close();
//}

void DataLogger::createLogFile(const std::string &filename, std::stringstream &header) { //TODO raise exception
    complete_file_path = complete_dir_path.append("/" + filename);
    fs.open(complete_file_path.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
    fs << header.str();
    fs.close();
}

void DataLogger::updateLogFile(const std::string &filename, const std::string &new_sample) {
    fs.open(complete_file_path.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
    fs << new_sample;
    fs.close();
}

void DataLogger::updateLogFile(const std::string &filename, std::stringstream &new_sample) {
    fs.open(complete_file_path.c_str(), std::fstream::in | std::fstream::app | std::fstream::out);
    fs << new_sample.str();
    fs.close();
}
