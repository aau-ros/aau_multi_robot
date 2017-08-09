#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

class DataLogger {
public:
    DataLogger();
    bool createLogFile();
    bool updateLogFile();
};

#endif // DATA_LOGGER_H
