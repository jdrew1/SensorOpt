#include "mainHeader.h"

int main(int argc, char **argv) {
    //init
    //==================================================================================================================
    InitProgram(argv);
	SetupCARLA();


    //start collecting training data
    //==================================================================================================================
    Eigen::MatrixX3f vehiclePoints = LiDAR::FetchVehiclePoints();
    //LiDAR::CheckPointsWithDebugVisualizer(vehiclePoints, true);

    //calculate the total lidar occupancy of each point on the vehicle for ground truth
    std::vector<int> TLOPerPoint;
    for (auto point : vehiclePoints.rowwise()){
        int tlo = LiDAR::CalculateTotalLidarOccupancy(LiDAR::FetchCylinderPoints(point.coeffRef(0),
                                                                                 point.coeffRef(1),
                                                                                 point.coeffRef(2)));
        TLOPerPoint.push_back(tlo);
    }

    //LiDAR::CheckPointsWithDebugVisualizer(LiDAR::FetchCylinderPoints(),false);

    //clean up
    //==================================================================================================================
    CloseCARLA();
    CleanProgram();
    return 0;
}
