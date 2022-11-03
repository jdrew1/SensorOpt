#include "savePoints.h"

namespace SavePoints{
    int SavePointsToDisc(Eigen::MatrixX3f pointsToSave, std::vector<int> totalLidarOccup, bool storeWithTLO, const char* filename){
        std::ofstream logFile;
        logFile.open(filename, std::ios::out);

        logFile << "NumOfPoints: " << pointsToSave.rows() << std::endl;
        for (int i = 0; i < pointsToSave.rows(); i++){
        logFile << pointsToSave.block(i,0,1,3) << "|";
        if (storeWithTLO)
            logFile << totalLidarOccup.at(i) << std::endl;
        }

        logFile.close();
        return true;
    }
    Eigen::MatrixX3f ReadPointsFromDisc(const char* filename, bool readTLO){

    }
}