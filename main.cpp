#include "mainHeader.h"



int main() {
    //init training data
    std::vector<Eigen::RowVectorXf*> images = ConvertMNISTImageData(FindInputData("../../../MNISTdataset/train/images"));
    std::vector<Eigen::RowVectorXf*> labels = ConvertMNISTLabelData(FindInputData("../../../MNISTdataset/train/labels"));

    //init network
    std::vector<int> topology = {(int)images[0]->size(), 100, 100, (int)labels[0]->size()};
    Perceptron imagesTest = Perceptron(topology, 0.1, Perceptron::Activation::sigmoid);
    //train
    PerformMNISTTrain(&imagesTest,images,labels);
    PerformMNISTTrain(&imagesTest,images,labels);
    PerformMNISTTrain(&imagesTest,images,labels);
    //init testing data
    std::vector<Eigen::RowVectorXf*> TESTlabels = ConvertMNISTLabelData(FindInputData("../../../MNISTdataset/test/labels"));
    std::vector<Eigen::RowVectorXf*> TESTimages = ConvertMNISTImageData(FindInputData("../../../MNISTdataset/test/images"));

    //test
    PerformMNISTTest(&imagesTest,TESTimages, TESTlabels);

    return 0;
}
