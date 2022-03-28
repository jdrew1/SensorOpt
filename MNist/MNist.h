#ifndef NEURALPROJECT_MNIST_H
#define NEURALPROJECT_MNIST_H

#include "../lib/Eigen/Eigen/Eigen"
#include <fstream>
#include <iostream>
#include <vector>
#include "../perceptron.h"


std::vector<Eigen::RowVectorXf*> ConvertMNISTImageData(std::ifstream dataFile);
std::vector<Eigen::RowVectorXf*> ConvertMNISTLabelData(std::ifstream dataFile);


void PerformMNISTTrain(Perceptron *network, std::vector<Eigen::RowVectorXf*> images, std::vector<Eigen::RowVectorXf*> labels);
void PerformMNISTTest(Perceptron *network, std::vector<Eigen::RowVectorXf*> images, std::vector<Eigen::RowVectorXf*> labels);


#endif //NEURALPROJECT_MNIST_H
