#ifndef NEURAL_PROJECT_LIDAR_H
#define NEURAL_PROJECT_LIDAR_H

#include <string>
#include <sstream>
#include "Eigen/Eigen"
#include "PythonAPI.h"
#include "network/perceptron.h"
#include "logger.h"

namespace LiDAR{
    Eigen::MatrixX3f FetchVehiclePoints(int numberOfPoints = 0);
    Eigen::MatrixX3f FetchCylinderPoints(float x = 0, float y = 0, float z = 0);
    void RunTest();

    Eigen::MatrixX3f CarlaToNetwork(PyObject * fromCarla, int networkInputSize = 0);
    std::string NetworkToCarla(Perceptron * network);
    std::string NetworkToCarla(float x, float y, float z);
    void ScalePointToVehicleBoundingBox(Perceptron * network, Eigen::RowVectorXf points);
    Eigen::MatrixX3f CalculatePointsOnCylinder(PyObject * fromCarla);
    void CheckPointsWithDebugVisualizer(Eigen::MatrixX3f pointsToCheck);
    int CalculateTotalLidarOccupancy(Eigen::MatrixX3f cylinderPoints);
}
#endif