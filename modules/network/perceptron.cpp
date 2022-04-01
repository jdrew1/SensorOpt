#include "perceptron.h"

using namespace Eigen;



Perceptron::Perceptron() {}
Perceptron::~Perceptron() {}
Perceptron::Perceptron(std::vector<int> shape, double learningR, enum Activation decisionFunction) {
    init(shape, learningR, decisionFunction);
}
void Perceptron::init(std::vector<int> shape, double learningR, enum Activation decisionFunction) {
    topology = shape;
    learningRate = learningR;
    activate = decisionFunction;
    neurons.clear();
    errors.clear();
    weights.clear();

    for(int i=0;i<topology.size();i++){
        //create each layer of the network, adding an extra neuron for all but the final layer
        //the extra neuron (=1) is the "bias" neuron, and acts as the y-intercept for the next layer
        neurons.push_back(new RowVectorXf(topology[i] + (i != (topology.size()-1))));
        errors.push_back(new RowVectorXf(topology[i] + (i != (topology.size()-1))));

        neurons.back()->setZero();
        //init the bias neuron to one
        if (i < topology.size()-1)
            neurons.back()->coeffRef(topology[i]) = 1.0;
        if (i > 0 ){
            weights.push_back(new MatrixXf(topology[i-1]+1,topology[i] + (i != (topology.size()-1))));
            weights.back()->setRandom();
        }
    }
}



double Perceptron::Activation(double x) {
    switch (activate){
        case tanH:
            return std::tanh(x);
        case sigmoid:
            return 1.0 / (1.0 + std::exp(-x));
        case abovezero:
            if (x > 0.0) return x;
            return 0.0;
        default:
            return 0;
    }
}
double Perceptron::ActivationDerivative(double x) {
    switch (activate) {
        case tanH:
            return 1 - std::tanh(x) * std::tanh(x);
        case sigmoid:
            return x * (1.0 - x);
        case abovezero:
            if (x > 0) return 1.0;
            return 0.0;
        default:
            return 0.0;
    }
}



void Perceptron::ForwardProp(Eigen::RowVectorXf &input) {
    //set the inputs (without disturbing the bias neuron)
    neurons.front()->block(0,0,1,input.size()) = input;

    //propagate forward
    for (int i=1;i<topology.size();i++){
        //do the forward calculation
        neurons[i]->block(0,0,1,topology[i]) = (*neurons[i-1] * *weights[i-1]).block(0,0,1,topology[i]);
        for (int j=0;j<topology[i];j++)
            neurons[i]->coeffRef(j) = Activation(neurons[i]->coeffRef(j));
    }
}
void Perceptron::BackwardProp(Eigen::RowVectorXf &correct) {
    //calculate the last layer's errors
    *errors.back() = correct - *neurons.back();

    //calculate the errors for each neuron going backward
    for (int i=errors.size()-2;i>0;i--)
        *errors[i] = *errors[i+1] * weights[i]->transpose();
    //update the weights according to the errors
    for (int i=0;i<weights.size();i++)
        for(int row=0;row<weights[i]->rows();row++)
            for(int col=0;col<weights[i]->cols();col++)
                weights[i]->coeffRef(row,col) +=
                        learningRate *
                        errors[i+1]->coeffRef(col) *
                        neurons[i]->coeffRef(row) *
                        ActivationDerivative(neurons[i+1]->coeffRef(col));
}



void Perceptron::Train(Eigen::RowVectorXf &input, Eigen::RowVectorXf &correct) {
    ForwardProp(input);
    BackwardProp(correct);
}

bool Perceptron::Test(Eigen::RowVectorXf &input, Eigen::RowVectorXf &correct) {
    ForwardProp(input);
    int guess = 0, corr = 0;
    double highestguess = 0, highestcorr = 0;
    for(int i=0; i<correct.size();i++){
        if (Output(i) > highestguess) {
            highestguess = Output(i);
            guess = i;
        }
        if (correct.coeffRef(i) > highestcorr) {
            highestcorr = correct.coeffRef(i);
            corr = i;
        }
    }
    if (guess == corr)
        return true;
    return false;
}


double Perceptron::MeanSquareError() {
    return std::sqrt((errors.back()->dot(*errors.back())) / errors.back()->size());
}



double Perceptron::Output(int col) {
    return (*neurons.back())[col];
}



bool Perceptron::Save(const char *filename) {
    try{
        std::stringstream tplgy;
        for (auto it = topology.begin(), _end = topology.end(); it != _end;it++)
            tplgy << *it << (it != _end ? "," : "");

        std::stringstream wghts;
        for (auto it = weights.begin(), _end = weights.end(); it != _end;it++)
            tplgy << *it << (it != _end ? "," : "") << std::endl;


        std::ofstream file(filename);
        file << "learningRate: " << learningRate << std::endl;
        file << "architecture: " << tplgy.str() << std::endl;
        file << "activation: " << activate << std::endl;
        file << "weights: " << std::endl << wghts.str() << std::endl;
        file.close();
    }
    catch(...){
        return false;
    }
    return true;
}



bool Perceptron::Load(const char *filename) {
    topology.clear();

    std::ifstream file(filename);
    if (!file.is_open())
        return false;
    std::string line, name, value;
    if (!getline(file, line, '\n'))
        return false;
    std::stringstream lr(line);

    // read learning rate
    getline(lr, name, ':');
    if (name != "learningRate")
        return false;
    if (!getline(lr, value, '\n'))
        return false;
    learningRate = atof(value.c_str());

    // read topology
    getline(file, line, '\n');
    std::stringstream ss(line);
    getline(ss, name, ':');
    if (name != "architecture")
        return false;
    while (getline(ss, value, ','))
        topology.push_back(atoi(value.c_str()));

    // read activation
    getline(file, line, '\n');
    std::stringstream sss(line);
    getline(sss, name, ':');
    if (name != "activation")
        return false;
    if (!getline(sss, value, '\n'))
        return false;
    activate = (enum Activation)atoi(value.c_str());

    // initialize using read architecture
    init(topology, learningRate, activate);

    // read weights
    getline(file, line, '\n');
    std::stringstream we(line);
    getline(we, name, ':');
    if (name !="weights")
        return false;

    std::string matrix;
    for (int i = 0; i < topology.size(); i++)
        if (getline(file, matrix, ',')) {
            std::stringstream ss(matrix);
            int row = 0;
            while (getline(ss, value, '\n'))
                if (!value.empty()) {
                    std::stringstream word(value);
                    int col = 0;
                    while (getline(word, value, ' '))
                        if (!value.empty())
                            weights[i]->coeffRef(row, col++) = atof(value.c_str());
                    row++;
                }
        }

    file.close();
    return true;




}
