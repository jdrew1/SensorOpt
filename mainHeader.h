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
    #include "network/perceptron.h"
    #include "settingsfile.h"
    #include "logger.h"
    #include "PythonAPI.h" 
    //application specific
    //--------------------------------------------------
    #include "modules/MNist/MNist.h"
    #include "modules/MNist/DataHandling.h"




    void InitProgram(){
        SettingsFile::InitSettings();
        MyLogger::InitLogging();
    }
#endif
