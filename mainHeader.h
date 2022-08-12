#ifndef MAINHEADER_H
#define MAINHEADER_H
    //standard libs
    //--------------------------------------------------
    #include <iostream>
    #include <vector>
    //dependencies
    //--------------------------------------------------
    #include "lib/Eigen/Eigen/Eigen"
    #include "lib/rapidJSON/include/rapidjson/rapidjson.h"
    //basic files
    //--------------------------------------------------
    #include "network/perceptron.hpp"
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
		CarlaAPI::SetupPython(arguments);

    }

    void CleanProgram() {
        CarlaAPI::ExitPython();
    }
#endif
