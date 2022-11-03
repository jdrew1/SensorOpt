#ifndef NEURALPROJECT_LOGGER_H
#define NEURALPROJECT_LOGGER_H

#include <iostream>
#include <fstream>
#include <ctime>
#include "settingsFile/settingsfile.h"


namespace MyLogger{
    static std::string logPath;
    enum LogType {Error=0, Warning=1, FATAL=2, IO=3, Message=4, Status=5};
    void InitLogging();
    bool SaveToLog(const char* message, LogType type);
}

#endif //NEURALPROJECT_LOGGER_H