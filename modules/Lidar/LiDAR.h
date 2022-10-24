#ifndef NEURAL_PROJECT_LIDAR_H
#define NEURAL_PROJECT_LIDAR_H

#include <string>
#include <sstream>
#include "Eigen/Eigen"
#include "PythonAPI.h"
#include "network/perceptron.h"
#include "logger.h"

namespace LiDAR{
    void SetupCARLA();
    void RunTest();
    void CloseCARLA();

    Eigen::RowVectorXf CarlaToNetwork(PyObject * fromCarla, int networkInputSize);
    std::string NetworkToCarla(Perceptron * network);
    void ScalePointToVehicleBoundingBox(Perceptron * network, Eigen::RowVectorXf points);
    Eigen::MatrixX3f CalculatePointsOnCylinder(PyObject * fromCarla, int randomSamplingSize = 10000);
    void CheckPointsWithDebugVisualizer(Eigen::MatrixXf pointsToCheck);
}
#endif