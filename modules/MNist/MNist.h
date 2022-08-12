#ifndef NEURALPROJECT_MNIST_H
#define NEURALPROJECT_MNIST_H

#include <fstream>
#include <iostream>
#include <vector>
#include "Eigen/Eigen"
#include "network/perceptron.hpp"

namespace MNist{
    std::vector<Eigen::RowVectorXf *> ConvertMNISTImageData(std::ifstream dataFile);
    std::vector<Eigen::RowVectorXf *> ConvertMNISTLabelData(std::ifstream dataFile);


    void PerformMNISTTrain(Perceptron *network, std::vector<Eigen::RowVectorXf *> images,
                           std::vector<Eigen::RowVectorXf *> labels);
    void PerformMNISTTest(Perceptron *network, std::vector<Eigen::RowVectorXf *> images,
                          std::vector<Eigen::RowVectorXf *> labels);

    std::ifstream FindInputData(const char* filename);
    int BasicMNISTPercept();

}
#endif //NEURALPROJECT_MNIST_H
