#ifndef MAINHEADER_H
#define MAINHEADER_H
    //standard libs
    //--------------------------------------------------
    #include <iostream>
    #include <vector>
    #include <stdio.h>
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
        // open the carla simulator
        //FILE * carla = popen(SettingsFile::StringSetting("pathToCarla").c_str(),"r");

    }

    void SetupCARLA(){
        PythonAPI::RunPyScript("parseArguments", "");
        PythonAPI::RunPyScript("setupEnvironment", "");
        PythonAPI::RunPyScript("place_cylinder_and_car",("D:" + SettingsFile::StringSetting("distanceToTestCylinder")));
    }

    void CloseCARLA(){
        //PythonAPI::RunPyScript("debugVisualizer","");
        PythonAPI::RunPyScript("closeEnvironment","");
    }

    void CleanProgram() {
        PythonAPI::ExitPython();
    }
#endif
