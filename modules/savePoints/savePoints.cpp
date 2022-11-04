#include "savePoints.h"

namespace SavePoints{
    int SavePointsToDisc(Eigen::MatrixX3f pointsToSave, std::vector<int> totalLidarOccup, bool storeWithTLO, const char* filename){
        std::ofstream logFile;
        logFile.open(filename, std::ios::out);

        logFile << "NumOfPoints:" << pointsToSave.rows() << std::endl;
        for (int i = 0; i < pointsToSave.rows(); i++){
        logFile << pointsToSave.coeffRef(i,0) << ","
                << pointsToSave.coeffRef(i,1) << ","
                << pointsToSave.coeffRef(i,2);
        if (storeWithTLO)
            logFile << "|" << totalLidarOccup.at(i);
        logFile << std::endl;
        }

        logFile.close();
        return true;
    }

    Eigen::MatrixX3f ReadPointsFromDisc(std::vector<int>* TotalLidarOccupancy, const char* filename, bool readTLO){
        std::ifstream inputReader;
        inputReader.open(filename, std::ios::in);
        if (!inputReader.is_open()){
            MyLogger::SaveToLog(("Unable to open points file at: " + (std::string)filename).c_str(), MyLogger::FATAL);
            return Eigen::MatrixX3f(1,3).setZero();
        }
        std::string singleLine;
        std::getline(inputReader,singleLine);
        if (singleLine.find("NumOfPoints:") == std::string::npos){
            MyLogger::SaveToLog(("Unknown points file format at: " + (std::string)filename).c_str(), MyLogger::FATAL);
            return Eigen::MatrixX3f(1,3).setZero();
        }

        Eigen::MatrixX3f output(std::stoi(singleLine.substr(strlen("NumOfPoints:"), singleLine.size() - strlen("NumOfPoints:"))),3);
        output.setZero();
        if (readTLO && TotalLidarOccupancy->size() != 0)
            TotalLidarOccupancy->clear();

        for(auto point : output.rowwise()){
            std::getline(inputReader, singleLine);
            if (singleLine.find('|') != std::string::npos && readTLO){
                TotalLidarOccupancy->push_back(std::stoi(singleLine.substr(singleLine.find('|')+1,singleLine.size()-singleLine.find('|')-1))+1);
            }
            point.coeffRef(0) = std::stof(singleLine.substr(0,singleLine.find(',')));
            point.coeffRef(1) = std::stof(singleLine.substr(singleLine.find(',')+1,singleLine.find_last_of(',')));
            point.coeffRef(2) = std::stof(singleLine.substr(singleLine.find_last_of(',')+1,
                                                          singleLine.find('|') == std::string::npos ?
                                                                    singleLine.size() - singleLine.find('|') - singleLine.find_last_of(',')
                                                                  : singleLine.size() - singleLine.find_last_of(',')-1));
        }
        inputReader.close();
        return output;


    }
}