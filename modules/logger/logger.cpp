#include "logger.h"

namespace MyLogger{
    using namespace std;
    void InitLogging(){
        time_t currentTime = time(0);
        logPath = SettingsFile::StringSetting("logfilePath");
        fstream logFile;
        logFile.open(logPath, ios::in);
        if (!logFile){
            logFile.open(logPath, ios::app);
            logFile << "Created new LogFile: " << ctime(&currentTime) << "----------------------------------------" << endl;
            std::cout << "Created new LogFile: " << ctime(&currentTime) << "----------------------------------------" << endl;
        }
        else{
            logFile.close();
            logFile.open(logPath, ios::app);
            logFile << std::endl << "New run of program: " << ctime(&currentTime) << "----------------------------------------" << endl;
            std::cout << std::endl << "New run of program: " << ctime(&currentTime) << "----------------------------------------" << endl;
        }
        logFile.close();
    }
    bool SaveToLog(const char* message, LogType type){
        ofstream logFile;
        logFile.open(logPath, ios::app);
        if (!logFile){
            cout << "ERROR: LOGFILE CHANGED OR REMOVED!" << endl;
            return false;
        }
        else{
            string preface;
            switch(type){
                case 0:
                    preface = "Error: \t";
                    break;
                case 1:
                    preface = "Warning: \t";
                    break;
                case 2:
                    preface = "FATAL: \n =============================================== \n\t";
                    break;
                case 3:
                    preface = "IO: \t";
                    break;
                case 4:
                    preface = "Message: \t";
                    break;
                case 5:
                    preface = "Status: \t";
                    break;
            }
            logFile << preface << message << endl;
            std::cout << preface << message << endl;
        }
        logFile.close();
        return true;
    }
}