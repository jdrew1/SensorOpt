#ifndef NEURAL_PROJECT_LIDAR_H
#define NEURAL_PROJECT_LIDAR_H

#include <string>
#include <sstream>
#include "Eigen/Eigen"
#include "pythonAPI/PythonAPI.h"
#include "network/perceptron.h"
#include "logger/logger.h"

namespace LiDAR{
    Eigen::MatrixX3f FetchVehiclePoints(int numberOfPoints = 0);
    Eigen::MatrixX3f FetchCylinderPoints(float x = 0, float y = 0, float z = 0);

    Eigen::MatrixX3f CarlaToNetwork(PyObject * fromCarla, int networkInputSize = 0);
    std::string NetworkToCarla(float x, float y, float z);
    Eigen::MatrixX3f CalculatePointsOnCylinder(PyObject * fromCarla);
    int CalculateTotalLidarOccupancy(Eigen::MatrixX3f cylinderPoints);
    void ScalePointToVehicleBoundingBox(Perceptron * network, Eigen::RowVectorXf points);
    void CheckPointsWithDebugVisualizer(Eigen::MatrixX3f pointsToCheck, bool cartesian);
}
#endif