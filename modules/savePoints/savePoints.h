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

namespace SavePoints{
    int SavePointsToDisc(Eigen::MatrixX3f pointsToSave,
                         std::vector<int> totalLidarOccup,
                         bool storeWithTLO,
                         const char* filename = SettingsFile::StringSetting("pointsFilePath").c_str());
    Eigen::MatrixX3f ReadPointsFromDisc(const char* filename = SettingsFile::StringSetting("pointsFilePath").c_str(),
                                        bool readTLO = false);
}


#endif //NEURALPROJECT_SAVEPOINTS_H