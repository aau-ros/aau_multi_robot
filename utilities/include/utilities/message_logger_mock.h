#ifndef MESSAGE_LOGGER_MOCK_H
#define MESSAGE_LOGGER_MOCK_H

#include <unordered_map>

#include "utilities/message_logger.h"

class MessageLoggerMock : public MessageLogger
{
public:
    MessageLoggerMock();
    void logDebug(std::stringstream stream);
//    void logInfo(std::stringstream stream);
//    void logWarn(std::stringstream stream);
//    void logError(std::stringstream stream);
//    void logFatal(std::stringstream stream);

private:
    std::unordered_map<std::string, unsigned int> calltrace;
};

#endif // MESSAGE_LOGGER_MOCK_H
