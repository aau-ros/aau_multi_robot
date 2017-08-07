#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

class DataLogger {
public:
    DataLogger();
    //DataLogger(std::string node_name, unsigned int robot_id);
    bool createLogFile();
    //bool createLogFile(std::string filename, std::stringstream header);
    bool updateLogFile();
    //bool updateLogFile(std::string filename, std::stringstream new_sample);
};

#endif // DATA_LOGGER_H
