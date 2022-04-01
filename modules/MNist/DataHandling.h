#ifndef NEURALPROJECT_DATAHANDLING_H
#define NEURALPROJECT_DATAHANDLING_H

#include <iostream>
#include <fstream>
#include "Eigen/Eigen"
#include "MNist.h"
namespace MNist{
    std::ifstream FindInputData(const char* filename){
        //for now, just search for the filename directly.
        //later, potentially implement settings file with data folder/names.
        std::ifstream attempt(filename, std::ios::binary | std::ios::in);
        if (!attempt) {
            std::cout << "Could not find file at: \"" << filename << "\"\n";
            throw std::invalid_argument("FileNotFound");
        }
        std::cout << "File opened successfully";
        return attempt;
    }
}
#endif //NEURALPROJECT_DATAHANDLING_H
