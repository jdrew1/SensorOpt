#include "LiDAR.h"

namespace LiDAR{

    void SetupCARLA(){
        PythonAPI::RunPyScript("parseArguments", "");
        PythonAPI::RunPyScript("setupEnvironment", "");
    }
    void RunTest(){
        std::vector<int> settingtopology;
        std::string layer;
        std::stringstream networkshape = std::stringstream(SettingsFile::StringSetting("networkShape"));
        while(getline(networkshape,layer, ':'))
            settingtopology.push_back(std::stoi(layer));
        Perceptron *network = new Perceptron(settingtopology);
        std::string carlaInput = NetworkToCarla(network);
        Eigen::RowVectorXf output = CarlaToNetwork(PythonAPI::RunPyScript("trainLoop",carlaInput));
    }
    void CloseCARLA(){
        PythonAPI::RunPyScript("closeEnvironment","");
    }

    Eigen::RowVectorXf CarlaToNetwork(PyObject * fromCarla){
        if(PythonAPI::GetPyObjectFormat(fromCarla) != "open3d.cpu.pybind.geometry.PointCloud") {
            MyLogger::SaveToLog(("Unexpected return type from Carla:"
                                + PythonAPI::GetPyObjectFormat(fromCarla)).c_str(),MyLogger::FATAL);
            return Eigen::RowVectorXf(0);
        }
        PyObject * pointCollection = PyObject_GetAttrString(fromCarla,"points");
        Eigen::RowVectorXf  output = Eigen::RowVectorXf(PySequence_Length(pointCollection)*3);
        PyObject * pointContainer;
        for(int i = 0; i < PySequence_Length(pointCollection); i ++){
            pointContainer = PySequence_GetItem(pointCollection, i);
            output.coeffRef(3*i) = PyFloat_AsDouble(PyObject_GetAttrString(pointContainer, "X"));
            output.coeffRef(3*i+1) = PyFloat_AsDouble(PyObject_GetAttrString(pointContainer, "Y"));
            output.coeffRef(3*i+2) = PyFloat_AsDouble(PyObject_GetAttrString(pointContainer, "Z"));
        }
        return output;

    }
    std::string NetworkToCarla(Perceptron * network){
        float xOffset = 0.0, yOffset = 0.0, zOffset = 0.0;
        if (network->topology.back() != 3){
            MyLogger::SaveToLog(("NetworkToCarla: Network output wrong dimension{3}: "
                                + std::to_string(network->topology.back())).c_str(),MyLogger::FATAL);
            return "";
        }
        xOffset = network->neurons.back()->coeffRef(0);
        yOffset = network->neurons.back()->coeffRef(1);
        zOffset = network->neurons.back()->coeffRef(2);

        return "--xOffset " + std::to_string(xOffset)
             + " --yOffset " + std::to_string(yOffset)
             + " --xOffset " + std::to_string(zOffset);
    }
}
