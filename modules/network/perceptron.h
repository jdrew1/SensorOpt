#ifndef NEURALPROJECT_PERCEPTRON_H
#define NEURALPROJECT_PERCEPTRON_H
#include <vector>
#include "Eigen/Eigen"
#include <math.h>
#include <fstream>
#include <string>


class Perceptron{
public:
    enum Activation {tanH, sigmoid, abovezero} activate;
    double learningRate = 0;
    std::vector<int> topology;
    std::vector<Eigen::RowVectorXf*> neurons;
    std::vector<Eigen::RowVectorXf*> errors;
    std::vector<Eigen::MatrixXf*> weights;

    Perceptron();
    ~Perceptron();
    Perceptron(std::vector<int> shape, double learningR = 0.05, Activation decisionFunction = sigmoid);
    void init(std::vector<int> shape, double learningR = 0.05, Activation decisionFunction = sigmoid);

    bool Load(const char* filename);
    bool Save (const char* filename);


    void ForwardProp(Eigen::RowVectorXf& input);
    double Activation(double x);

    void BackwardProp(Eigen::RowVectorXf& correct);
    double ActivationDerivative(double x);

    void Train(Eigen::RowVectorXf& input, Eigen::RowVectorXf& correct);
    bool Test(Eigen::RowVectorXf& input, Eigen::RowVectorXf& correct);

    double Output(int col);

    double MeanSquareError();
};






#endif //NEURALPROJECT_PERCEPTRON_H
