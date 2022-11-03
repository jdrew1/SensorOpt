#include "mainHeader.h"

int main(int argc, char **argv) {
    InitProgram(argv);
	SetupCARLA();



    Eigen::MatrixX3f vehiclePoints = LiDAR::FetchVehiclePoints();
    Eigen::MatrixX3f cylinderPoints = LiDAR::FetchCylinderPoints();
    //std::vector<int> TLOPerPoint;
    //TLOPerPoint.push_back(LiDAR::CalculateTotalLidarOccupancy(cylinderPoints));

    LiDAR::CheckPointsWithDebugVisualizer(vehiclePoints, true);
    LiDAR::CheckPointsWithDebugVisualizer(cylinderPoints, false);




    CloseCARLA();
    CleanProgram();
    return 0;
}
