#ifndef MESSAGE_LOGGER_H
#define MESSAGE_LOGGER_H

#include <sstream>
#include <ros/ros.h>

class MessageLogger
{
public:
    virtual void logDebug(std::stringstream stream) = 0;
//    virtual void logInfo(std::stringstream stream) = 0;
//    virtual void logWarn(std::stringstream stream) = 0;
//    virtual void logError(std::stringstream stream) = 0;
//    virtual void logFatal(std::stringstream stream) = 0;
};

#endif // MESSAGE_LOGGER_H
