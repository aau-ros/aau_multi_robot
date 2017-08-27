#include "utilities/message_logger_concrete.h"

ConcMessageLogger::ConcMessageLogger() {}

void ConcMessageLogger::logDebug(std::stringstream stream) {
    ROS_DEBUG_STREAM(stream);
}
