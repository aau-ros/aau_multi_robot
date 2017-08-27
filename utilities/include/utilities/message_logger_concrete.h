#ifndef MESSAGE_LOGGER_CONCRETE_H
#define MESSAGE_LOGGER_CONCRETE_H

#include "utilities/message_logger.h"

class ConcMessageLogger : public MessageLogger
{
public:
    ConcMessageLogger();
    void logDebug(std::stringstream stream);
//    void logInfo(std::stringstream stream);
//    void logWarn(std::stringstream stream);
//    void logError(std::stringstream stream);
//    void logFatal(std::stringstream stream);
};

#endif // MESSAGE_LOGGER_CONCRETE_H
