#include "mainHeader.h"

int main(int argc, char **argv) {
    InitProgram(argv);
	SetupCARLA();



    Eigen::MatrixX3f vehiclePoints = LiDAR::FetchVehiclePoints(100);
    std::vector<int> TLOPerPoint;
    for (auto point : vehiclePoints.rowwise()){
        TLOPerPoint.push_back(LiDAR::CalculateTotalLidarOccupancy(LiDAR::FetchCylinderPoints(point.coeffRef(0), point.coeffRef(1), point.coeffRef(2))));
    }

    for(int i = 0; i < vehiclePoints.rows(); i ++)
        std::cout << vehiclePoints.row(i) << "|" << TLOPerPoint.at(i) << std::endl;




    CloseCARLA();
    CleanProgram();
    return 0;
}
