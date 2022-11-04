#include "mainHeader.h"

int main(int argc, char **argv) {
    //init
    //==================================================================================================================
    InitProgram(argv);
    MyLogger::SaveToLog("Connecting to CARLA...",MyLogger::Message);
	SetupCARLA();


    //start collecting training data
    //==================================================================================================================
    MyLogger::SaveToLog("Fetching Points of Vehicle...", MyLogger::Message);
    Eigen::MatrixX3f vehiclePoints = LiDAR::FetchVehiclePoints();
    //LiDAR::CheckPointsWithDebugVisualizer(vehiclePoints, true);
    //LiDAR::CheckPointsWithDebugVisualizer(LiDAR::FetchCylinderPoints(),false);

    //calculate the total lidar occupancy of each point on the vehicle for ground truth
    MyLogger::SaveToLog(("Calculating ground truth of vehicle points: " + std::to_string(vehiclePoints.rows())).c_str(), MyLogger::Message);
    std::vector<int> TLOPerPoint;
    for (auto point : vehiclePoints.rowwise()){\
        int tlo = LiDAR::CalculateTotalLidarOccupancy(LiDAR::FetchCylinderPoints(point.coeffRef(0),
                                                                                 point.coeffRef(1),
                                                                                 point.coeffRef(2)));
        TLOPerPoint.push_back(tlo);
    }

    MyLogger::SaveToLog(("Saving points with TotalLidarOccupancy to file: " + SettingsFile::StringSetting("pointsFilePath")).c_str(), MyLogger::Message);
    SavePoints::SavePointsToDisc(vehiclePoints, TLOPerPoint, true);

    Eigen::MatrixX3f retrievedPoints = SavePoints::ReadPointsFromDisc(&TLOPerPoint);


    //clean up
    //==================================================================================================================
    MyLogger::SaveToLog("Cleaning up", MyLogger::Message);
    CloseCARLA();
    CleanProgram();
    return 0;
}
