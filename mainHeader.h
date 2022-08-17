#ifndef MAINHEADER_H
#define MAINHEADER_H
    //standard libs
    //--------------------------------------------------
    #include <iostream>
    #include <vector>
    #include <stdio.h>
    #ifdef __APPLE__
        #ifdef TARGET_OS_MAC
        //#include <>
        #endif
    #elif _WIN32
    #endif
    //dependencies
    //--------------------------------------------------
    #include "lib/Eigen/Eigen/Eigen"
    #include "lib/rapidJSON/include/rapidjson/rapidjson.h"
    //basic files
    //--------------------------------------------------
    #include "network/perceptron.h"
    #include "settingsfile.h"
    #include "logger.h"
    #include "PythonAPI.h"
    //application specific
    //--------------------------------------------------
    #include "modules/MNist/MNist.h"
    #include "modules/MNist/DataHandling.h"
    #include "Lidar/LiDAR.h"




    void InitProgram(char ** arguments){
        SettingsFile::InitSettings();
        MyLogger::InitLogging();
		PythonAPI::SetupPython(arguments);
        //FILE * carla = popen(SettingsFile::StringSetting("pathToCarla").c_str(),"r");

    }

    void CleanProgram() {
        PythonAPI::ExitPython();
    }
#endif
