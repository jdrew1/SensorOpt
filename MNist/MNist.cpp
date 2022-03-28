#include "MNist.h"
using namespace Eigen;
using namespace std;


vector<RowVectorXf*> ConvertMNISTImageData(ifstream dataFile){
    //init output
    std::vector<Eigen::RowVectorXf*> output;
    //collect magic number, number of images, rows, and columns
    uint header[4] = {0,0,0,0};
    dataFile.seekg(0);
    for(int i=0;i<4;i++){
        for(int j=3;j>=0;j--)
            header[i] += dataFile.get() <<j;
    }
    uint numOfPixelsPerImage = header[2] * header[3];
    //parse the actual data
    for(int i=0;i<header[1];i++){
        output.push_back(new Eigen::RowVectorXf(numOfPixelsPerImage));
        for(int j=0;j<numOfPixelsPerImage;j++) {
            if (dataFile.eof())
                throw std::length_error("Unhandled end of file");
            int a = dataFile.get();
            output.back()->coeffRef(j) = ((float)a)/255.0;
        }
    }
    return output;
}
std::vector<Eigen::RowVectorXf*> ConvertMNISTLabelData(std::ifstream dataFile){
    //init output
    std::vector<Eigen::RowVectorXf*> output;
    //collect magic number, number of images, rows, and columns
    uint header[2] = {0,0};
    dataFile.seekg(0);
    for(int i=0;i<2;i++){
        for(int j=3;j>=0;j--)
            header[i] += dataFile.get() <<j;
    }
    //parse the actual data
    for(int i=0;i<header[1];i++){
        output.push_back(new Eigen::RowVectorXf(10));
        output.back()->setZero();
        if (dataFile.eof())
            throw std::length_error("Unhandled end of file");
        int a = dataFile.get();
        output.back()->coeffRef(a-1) = 1;
    }
    return output;
}


void PerformMNISTTrain(Perceptron *network, vector<RowVectorXf*> images, vector<RowVectorXf*> labels){
    if (network->topology[0] != images[0]->size()){
        cout << "ERROR: Mismatch between input data size and network input size" << endl;
        return;
    }
    for (int i=0;i<images.size();i++){
        network->Train(*images[i],*labels[i]);
        cout << "After: " << i << " training images, MSE:" << endl << network->MeanSquareError() << endl;
    }
}

void PerformMNISTTest(Perceptron *network, std::vector<Eigen::RowVectorXf*> images, std::vector<Eigen::RowVectorXf*> labels){
    if (network->topology[0] != images[0]->size()){
        cout << "ERROR: Mismatch between input data size and network input size" << endl;
        return;
    }
    int numbercorrect;
    for (int i=0;i<images.size();i++){
        bool result = network->Test(*images[i],*labels[i]);
        cout << *network->neurons.back() << "\n" <<*labels[i] << "\n";
        numbercorrect += result;
        cout << "Iteration " << i << "\n" << result << "\n";
        cout << "Running SuccessRate: "<< (double)(numbercorrect) / (double)(i) << "          " << numbercorrect <<  "\n";
    }
}
