#include "LiDAR.h"

namespace LiDAR{

    void SetupCARLA(){
        PythonAPI::RunPyScript("parseArguments", "");
        PythonAPI::RunPyScript("setupEnvironment", "");
        PythonAPI::RunPyScript("place_cylinder_and_car",("D:" + SettingsFile::StringSetting("distanceToTestCylinder")));
    }
    void RunTest(){
        //init network
        std::vector<int> settingTopology;
        std::string layer;
        std::stringstream networkShape = std::stringstream(SettingsFile::StringSetting("networkShape"));
        while(getline(networkShape, layer, ':'))
            settingTopology.push_back(std::stoi(layer));
        //multiply the first layer by three to separate the x,y,z coordinates
        settingTopology.front() = settingTopology.front()*3;
        Perceptron *network = new Perceptron(settingTopology);
        //fetch the input to the network (point cloud of vehicle)
        Eigen::RowVectorXf carMesh = CarlaToNetwork(
                PythonAPI::RunPyScript("find_car_mesh", ""),
                network->topology.front());

        std::string carlaInput = NetworkToCarla(network);
    }
    void CloseCARLA(){
        PythonAPI::RunPyScript("closeEnvironment","");
    }

    Eigen::RowVectorXf CarlaToNetwork(PyObject * fromCarla, int networkInputSize){
        //check the format of the input object
        if(PythonAPI::GetPyObjectFormat(fromCarla) != "open3d.cpu.pybind.geometry.PointCloud") {
            MyLogger::SaveToLog(("Unexpected return type from Carla:"
                                + PythonAPI::GetPyObjectFormat(fromCarla)).c_str(),MyLogger::FATAL);
            return Eigen::RowVectorXf(0);
        }
        //extract the points from the point cloud object
        PyObject * pointCollection = PyObject_GetAttrString(fromCarla,"points");
        //init the eigen vector to pass to the network
        Eigen::RowVectorXf  output = Eigen::RowVectorXf(PySequence_Length(pointCollection)*3);
        PyObject * pointContainer;
        //extract each point
        for(int i = 0; i < PySequence_Length(pointCollection); i ++){
            pointContainer = PySequence_GetItem(pointCollection, i);
            output.coeffRef(3*i) = PyFloat_AsDouble(PyObject_GetAttrString(pointContainer, "X"));
            output.coeffRef(3*i+1) = PyFloat_AsDouble(PyObject_GetAttrString(pointContainer, "Y"));
            output.coeffRef(3*i+2) = PyFloat_AsDouble(PyObject_GetAttrString(pointContainer, "Z"));
        }
        //trim the eigen vector to have the same size as the network
        std::srand(std::time(0));
        while (output.size() > networkInputSize){
            int randIndex = std::rand() % ((networkInputSize/3)-1);
            Eigen::RowVectorXf tempA = output.block(0,0,1,randIndex*3);
            Eigen::RowVectorXf tempB = output.block(0,randIndex*3+3,output.rows(),output.cols()-(randIndex*3+3));
            output = Eigen::RowVectorXf(tempA.cols() + tempB.cols());
            output << tempA, tempB;
        }
        while (output.size() < networkInputSize){
            Eigen::RowVectorXf temp = Eigen::RowVectorXf(networkInputSize);
            temp << output, Eigen::RowVectorXf::Zero((networkInputSize) - output.size());
            output = temp;
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

        return "X:" + std::to_string(xOffset)
             + ",Y:" + std::to_string(yOffset)
             + ",Z:" + std::to_string(zOffset);
    }
}
