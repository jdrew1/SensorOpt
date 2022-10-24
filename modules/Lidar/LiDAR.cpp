#include "LiDAR.h"

namespace LiDAR{

    void SetupCARLA(){
        PythonAPI::RunPyScript("parseArguments", "");
        PythonAPI::RunPyScript("setupEnvironment", "");
        PythonAPI::RunPyScript("place_cylinder_and_car",("D:" + SettingsFile::StringSetting("distanceToTestCylinder")));
    }

    void RunTest(){
        //init network
        //--------------------------------------------------------------------------------------------------------------
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
        //--------------------------------------------------------------------------------------------------------------
        //start the training loop
        //--------------------------------------------------------------------------------------------------------------
        //fetch the input to the network (point cloud of vehicle)
        Eigen::RowVectorXf carMesh = CarlaToNetwork(
                                                    PythonAPI::RunPyScript("find_car_mesh", ""),
                                                    network->topology.front());
        CheckPointsWithDebugVisualizer(carMesh);
        //use the network to determine the output
        network->ForwardProp(carMesh);
        //network only returns [-1,1] so scale the points according to the bounding box of the car
        ScalePointToVehicleBoundingBox(network, carMesh);

        std::string carlaInput = NetworkToCarla(network);
        PythonAPI::RunPyScript("place_sensors",carlaInput);

        //retrieve the point cloud collected from the sensor
        PyObject* lidarMeasurement = PythonAPI::RunPyScript("fetch_lidar_measurement","");
        //and convert to cylindrical coordinates
        Eigen::MatrixX3f cylinderMesh = CalculatePointsOnCylinder(lidarMeasurement);
        CheckPointsWithDebugVisualizer(cylinderMesh);
        //calc total lidar occupancy
        //back-propogate
        //clean for next iteration
        //iterate
        PythonAPI::RunPyScript("debugVisualizer","");
    }

    void CloseCARLA(){
        //PythonAPI::RunPyScript("debugVisualizer","");
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
            output.coeffRef(3*i) = PyFloat_AsDouble(PySequence_GetItem(pointContainer,0));
            output.coeffRef(3*i+1) = PyFloat_AsDouble(PySequence_GetItem(pointContainer,1));
            output.coeffRef(3*i+2) = PyFloat_AsDouble(PySequence_GetItem(pointContainer,2));
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

    Eigen::MatrixX3f CalculatePointsOnCylinder(PyObject * fromCarla, int randomSamplingSize){
        //check the format of the input object
        if(PythonAPI::GetPyObjectFormat(fromCarla) != "open3d.cpu.pybind.geometry.PointCloud") {
            MyLogger::SaveToLog(("Unexpected return type from Carla:"
                                 + PythonAPI::GetPyObjectFormat(fromCarla)).c_str(),MyLogger::FATAL);
            return Eigen::RowVectorXf(0);
        }
        //extract the points from the point cloud object
        PyObject * pointCollection = PyObject_GetAttrString(fromCarla,"points");
        int numOfPoints = PySequence_Length(pointCollection);
        //init the eigen vector to pass to the network
        Eigen::MatrixX3f  cylindricalPoints = Eigen::MatrixX3f(PySequence_Length(pointCollection),3);
        PyObject * pointContainer;

        int cylIndex = 0;
        //extract each point
        for(int i = 0; i < numOfPoints; i ++){
            pointContainer = PySequence_GetItem(pointCollection, i);
            cylindricalPoints.coeffRef(cylIndex, 0) = sqrt(pow(PyFloat_AsDouble(PySequence_GetItem(pointContainer, 0)), 2)
                                                  + pow(PyFloat_AsDouble(PySequence_GetItem(pointContainer, 1)), 2));
            cylindricalPoints.coeffRef(cylIndex, 1) = atan2(PyFloat_AsDouble(PySequence_GetItem(pointContainer, 1))
                                                    ,
                                                    PyFloat_AsDouble(PySequence_GetItem(pointContainer, 0)))
                                                            +
                    (PyFloat_AsDouble(PySequence_GetItem(pointContainer, 1)) < 0 ? 2*M_PI : 0);
            cylindricalPoints.coeffRef(cylIndex, 2) = PyFloat_AsDouble(PySequence_GetItem(pointContainer, 2));
            if (cylindricalPoints.coeffRef(cylIndex,0) > 9.0 && cylindricalPoints.coeffRef(cylIndex,0) < 12.0)
                cylIndex++;
        }
        //discard any that aren't on the cylinder
        cylindricalPoints.conservativeResize(cylIndex+1,3);
        return cylindricalPoints;
    }

    void ScalePointToVehicleBoundingBox(Perceptron * network, Eigen::RowVectorXf points){
        int numberOfLidar = std::stoi(SettingsFile::StringSetting("numberOfLiDAR"));
        float x, y, z;
        PyObject* boundingBox = PythonAPI::RunPyScript("fetch_vehicle_bounding_box","");
        x = PyFloat_AsDouble(PyObject_GetAttrString(boundingBox,"x"));
        y = PyFloat_AsDouble(PyObject_GetAttrString(boundingBox,"y"));
        z = PyFloat_AsDouble(PyObject_GetAttrString(boundingBox,"z"));

        for (int i = 0; i < numberOfLidar; i ++){
            network->neurons.back()->coeffRef(3*i)   = network->neurons.back()->coeffRef(3*i)   * x*2;
            network->neurons.back()->coeffRef(3*i+1) = network->neurons.back()->coeffRef(3*i+1) * y*2;
            network->neurons.back()->coeffRef(3*i+2) = network->neurons.back()->coeffRef(3*i+2) * z*2;
        }
    }
    void CheckPointsWithDebugVisualizer(Eigen::MatrixXf pointsToCheck){
        std::string toCarla = "";
        switch(pointsToCheck.cols()){
            case 3:
                for(auto Point: pointsToCheck.rowwise()){
                    toCarla += "|";
                    toCarla += std::to_string(Point.coeffRef(0)* cos(Point.coeffRef(1)));
                    toCarla += ",";
                    toCarla += std::to_string(Point.coeffRef(0)* sin(Point.coeffRef(1)));
                    toCarla += ",";
                    toCarla += std::to_string(Point.coeffRef(2));
                }
                break;
            default:
                for(int i = 0; i < pointsToCheck.cols(); i+=3){
                    toCarla += "|";
                    toCarla += std::to_string(pointsToCheck.coeffRef(0,i));
                    toCarla += ",";
                    toCarla += std::to_string(pointsToCheck.coeffRef(0,i+1));
                    toCarla += ",";
                    toCarla += std::to_string(pointsToCheck.coeffRef(0,i+2));
                }
                break;
        }
        PythonAPI::RunPyScript("debugVisualizer", toCarla);
    }
}
