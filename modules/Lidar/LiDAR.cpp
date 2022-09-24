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
        //make sure the final layer of the network has correct length
        settingTopology.push_back(std::stoi(SettingsFile::StringSetting("numberOfLiDAR"))*3);
        Perceptron *network = new Perceptron(settingTopology);
        //fetch the input to the network (point cloud of vehicle)
        Eigen::RowVectorXf carMesh = CarlaToNetwork(
                                                    PythonAPI::RunPyScript("find_car_mesh", ""),
                                                    network->topology.front());
        //use the network to determine the output
        network->ForwardProp(carMesh);
        //network only returns [-1,1] so scale the points according to the bounding box of the car
        ScalePointToVehicleBoundingBox(network, carMesh);

        std::string carlaInput = NetworkToCarla(network);
        PythonAPI::RunPyScript("place_sensors",carlaInput);
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
        std::srand(std::time(0));
        //trim the eigen vector to have the same size as the network
        while (output.size() > networkInputSize){
            //pick a random point to avoid biasing the front of the array
            int randIndex = std::rand() % ((networkInputSize/3)-1);
            Eigen::RowVectorXf tempA = output.block(0,0,output.rows(),randIndex*3);
            Eigen::RowVectorXf tempB = output.block(0,randIndex*3+3,output.rows(),output.cols()-(randIndex*3+3));
            output = Eigen::RowVectorXf(tempA.cols() + tempB.cols());
            output << tempA, tempB;
        }
        //otherwise pad the vector with zeroes
        while (output.size() < networkInputSize){
            //pick a random point to avoid biasing the front of the array
            int randIndex = std::rand() % ((output.size()/3)-1);
            Eigen::RowVectorXf tempA = output.block(0,0,output.rows(),randIndex*3);
            Eigen::RowVectorXf tempB = output.block(0,randIndex*3,output.rows(),output.cols()-(randIndex*3));
            output = Eigen::RowVectorXf(tempA.cols() + tempB.cols() + 3);
            output << tempA, Eigen::RowVectorXf::Zero(3), tempB;
        }
        return output;
    }
    std::string NetworkToCarla(Perceptron * network){
        int numberOfLidar = std::stoi(SettingsFile::StringSetting("numberOfLiDAR"));
        if (network->topology.back() != numberOfLidar*3){
            MyLogger::SaveToLog(("NetworkToCarla: Network output wrong dimension{"
                                + std::to_string(numberOfLidar*3)
                                + "}: "
                                + std::to_string(network->topology.back())).c_str(),MyLogger::FATAL);
            return "";
        }
        std::string output;
        output += std::to_string(network->neurons.back()->coeffRef(0));
        output += "," + std::to_string(network->neurons.back()->coeffRef(1));
        output += "," + std::to_string(network->neurons.back()->coeffRef(2));
        for (int i = 1; i < numberOfLidar; i++){
            output += "|";
            output += "" + std::to_string(network->neurons.back()->coeffRef(i*3+0));
            output += "," + std::to_string(network->neurons.back()->coeffRef(i*3+1));
            output += "," + std::to_string(network->neurons.back()->coeffRef(i*3+2));
        }
        return output;
    }
    void ScalePointToVehicleBoundingBox(Perceptron * network, Eigen::RowVectorXf points){
        int numberOfLidar = std::stoi(SettingsFile::StringSetting("numberOfLiDAR"));
        float xMax, yMax, zMax, xMin, yMin, zMin, x, y, z;
        xMax = xMin = points.coeffRef(0);
        yMax = yMin = points.coeffRef(1);
        zMax = zMin = points.coeffRef(2);

        std::cout << "xmin: " << xMin << " : " << xMax << std::endl;
        std::cout << "ymin: " << yMin << " : " << yMax << std::endl;
        std::cout << "zmin: " << zMin << " : " << zMax << std::endl;
        std::cout << "After: " <<std::endl;
        for (int i =3; i < points.size(); i +=3){
            if (points.coeffRef(i) < xMin)   xMin = points.coeffRef(i);
            if (points.coeffRef(i+1) < yMin) yMin = points.coeffRef(i+1);
            if (points.coeffRef(i+2) < zMin) zMin = points.coeffRef(i+2);
            if (points.coeffRef(i) > xMax)   xMax = points.coeffRef(i);
            if (points.coeffRef(i+1) > yMax) yMax = points.coeffRef(i+1);
            if (points.coeffRef(i+2) > zMax) zMax = points.coeffRef(i+2);

        }
        std::cout << "xmin: " << xMin << " : " << xMax << std::endl;
        std::cout << "ymin: " << yMin << " : " << yMax << std::endl;
        std::cout << "zmin: " << zMin << " : " << zMax << std::endl;
        x = xMax - xMin;
        y = yMax - yMin;
        z = zMax - zMin;
        std::cout << "x range: " << x << std::endl;
        std::cout << "y range: " << y << std::endl;
        std::cout << "z range: " << z << std::endl;
        for (int i = 0; i < numberOfLidar; i ++){
            std::cout << "number of lidar points: " << numberOfLidar << std::endl;
            std::cout << "before scale : " << network->neurons.back()->coeffRef(0) << network->neurons.back()->coeffRef(1) << network->neurons.back()->coeffRef(2) << std::endl;
            network->neurons.back()->coeffRef(3*i)   = network->neurons.back()->coeffRef(3*i)   * x;
            network->neurons.back()->coeffRef(3*i+1) = network->neurons.back()->coeffRef(3*i+1) * y;
            network->neurons.back()->coeffRef(3*i+2) = network->neurons.back()->coeffRef(3*i+2) * z;

            std::cout << "after scale : " << network->neurons.back()->coeffRef(0) << network->neurons.back()->coeffRef(1) << network->neurons.back()->coeffRef(2) << std::endl;
        }
    }
}
