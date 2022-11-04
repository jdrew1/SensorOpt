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
    #include "settingsFile/settingsfile.h"
    #include "logger/logger.h"
    #include "pythonAPI/PythonAPI.h"
    #include "savePoints/savePoints.h"

    //application specific
    //--------------------------------------------------
    #include "Lidar/LiDAR.h"




    void InitProgram(char ** arguments){
        SettingsFile::InitSettings();
        MyLogger::InitLogging();
		PythonAPI::SetupPython(arguments);
        // open the carla simulator
        //FILE * carla = popen(SettingsFile::StringSetting("pathToCarla").c_str(),"r");

    }

    void SetupCARLA(){
        PythonAPI::RunPyScript("parseArguments", "", SettingsFile::StringSetting("carlaPyScriptLoc"));
        PythonAPI::RunPyScript("setupEnvironment", "", SettingsFile::StringSetting("carlaPyScriptLoc"));
        PythonAPI::RunPyScript("place_cylinder_and_car",
                               ("D:" + SettingsFile::StringSetting("distanceToTestCylinder")),
                               SettingsFile::StringSetting("carlaPyScriptLoc"));
    }

    void CloseCARLA(){
        //PythonAPI::RunPyScript("debugVisualizer","");
        PythonAPI::RunPyScript("closeEnvironment","", SettingsFile::StringSetting("carlaPyScriptLoc"));
    }

    void CleanProgram() {
        PythonAPI::ExitPython();
    }
#endif
