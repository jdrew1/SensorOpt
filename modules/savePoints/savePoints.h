#ifndef NEURALPROJECT_SAVEPOINTS_H
#define NEURALPROJECT_SAVEPOINTS_H
#include <Python.h>
#include <stdio.h>
#include <string>
#include <iostream>
#include <fstream>
#include <ctime>
#include "Eigen/Eigen"
#include "settingsFile/settingsfile.h"
#include "logger/logger.h"

namespace SavePoints{
    int SavePointsToDisc(Eigen::MatrixX3f pointsToSave,
                         std::vector<int> totalLidarOccup,
                         bool storeWithTLO,
                         const char* filename = SettingsFile::StringSetting("pointsFilePath").c_str());
    Eigen::MatrixX3f ReadPointsFromDisc(std::vector<int>* TotalLidarOccupancy,
                                        const char* filename = SettingsFile::StringSetting("pointsFilePath").c_str(),
                                        bool readTLO = true);
}


#endif //NEURALPROJECT_SAVEPOINTS_H