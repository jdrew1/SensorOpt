#include "LiDAR.h"

namespace LiDAR{

    Eigen::MatrixX3f FetchVehiclePoints(int numberOfPoints){
        return CarlaToNetwork(PythonAPI::RunPyScript("find_car_mesh", ""),
                              numberOfPoints);
    }


    Eigen::MatrixX3f FetchCylinderPoints(float x, float y, float z){
        std::string carlaInput = NetworkToCarla(x,y,z);
        PythonAPI::RunPyScript("place_sensors",carlaInput);

        //retrieve the point cloud collected from the sensor
        PyObject* lidarMeasurement = PythonAPI::RunPyScript("fetch_lidar_measurement","");
        //and convert to cylindrical coordinates
        return CalculatePointsOnCylinder(lidarMeasurement);
    }

    Eigen::MatrixX3f CarlaToNetwork(PyObject * fromCarla, int networkInputSize){
        //check the format of the input object
        if(PythonAPI::GetPyObjectFormat(fromCarla) != "open3d.cpu.pybind.geometry.PointCloud") {
            MyLogger::SaveToLog(("Unexpected return type from Carla:"
                                + PythonAPI::GetPyObjectFormat(fromCarla)).c_str(),MyLogger::FATAL);
            return Eigen::RowVectorXf(0);
        }
        //extract the points from the point cloud object
        PyObject * pointCollection = PyObject_GetAttrString(fromCarla,"points");
        //init the eigen vector to pass to the network
        Eigen::MatrixX3f output = Eigen::MatrixX3f(PySequence_Length(pointCollection),3);
        PyObject * pointContainer;
        //extract each point
        for(int i = 0; i < PySequence_Length(pointCollection); i ++){
            pointContainer = PySequence_GetItem(pointCollection, i);
            output.coeffRef(i,0) = PyFloat_AsDouble(PySequence_GetItem(pointContainer,0));
            output.coeffRef(i,1) = PyFloat_AsDouble(PySequence_GetItem(pointContainer,1));
            output.coeffRef(i,2) = PyFloat_AsDouble(PySequence_GetItem(pointContainer,2));
        }
        // if there is a defined amount of points, make sure the correct number is returned
        if (networkInputSize != 0){
            std::srand(std::time(0));
            //trim the eigen vector to have the same size as the network
            while (output.rows() > networkInputSize){
                //pick a random point to avoid biasing the front of the array
                int randIndex = std::rand() % networkInputSize;
                Eigen::MatrixX3f tempA = output.block(0,0,randIndex,3);
                Eigen::MatrixX3f tempB = output.block(randIndex+1,0,output.rows()-randIndex-1,output.cols());
                output = Eigen::MatrixX3f(tempA.rows() + tempB.rows(),3);
                output << tempA, tempB;
            }
            //otherwise pad the vector with zeroes
            while (output.rows() < networkInputSize){
                //pick a random point to avoid biasing the front of the array
                int randIndex = std::rand() % output.rows();
                Eigen::MatrixX3f tempA = output.block(0,0,randIndex,3);
                Eigen::MatrixX3f tempB = output.block(randIndex,0,output.rows()-randIndex,output.cols());
                output = Eigen::MatrixX3f(tempA.rows() + tempB.rows() + 1,3);
                output << tempA, Eigen::RowVectorXf::Zero(3), tempB;
            }
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

    std::string NetworkToCarla(float x, float y, float z){
        std::string output;
        output += std::to_string(x);
        output += "," + std::to_string(y);
        output += "," + std::to_string(z);
        return output;
    }

    Eigen::MatrixX3f CalculatePointsOnCylinder(PyObject * fromCarla){
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

    void CheckPointsWithDebugVisualizer(Eigen::MatrixX3f pointsToCheck, bool cartesian){
        std::string toCarla = "";
        if (cartesian){
            for(auto Point: pointsToCheck.rowwise()){
                toCarla += "|";
                toCarla += std::to_string(Point.coeffRef(0));
                toCarla += ",";
                toCarla += std::to_string(Point.coeffRef(1));
                toCarla += ",";
                toCarla += std::to_string(Point.coeffRef(2));
            }
        }
        else{
            for(auto Point: pointsToCheck.rowwise()){
                toCarla += "|";
                toCarla += std::to_string(Point.coeffRef(0)* cos(Point.coeffRef(1)));
                toCarla += ",";
                toCarla += std::to_string(Point.coeffRef(0)* sin(Point.coeffRef(1)));
                toCarla += ",";
                toCarla += std::to_string(Point.coeffRef(2));
            }
        }
        PythonAPI::RunPyScript("debugVisualizer", toCarla);
    }

    int CalculateTotalLidarOccupancy(Eigen::MatrixX3f cylinderPoints){
        float divisionsPerUnit = 10.0, height = 7.0, distanceToCylinder = std::stof(SettingsFile::StringSetting("distanceToTestCylinder"));
        Eigen::MatrixXi cylinderDivisions = Eigen::MatrixXi((int)(height * divisionsPerUnit), (int)((M_PI * 2 * distanceToCylinder) * divisionsPerUnit));
        cylinderDivisions.setZero();

        for (auto point : cylinderPoints.rowwise()){
            //place a '1' in each element of the matrix corresponding to a point found via Lidar
            cylinderDivisions.coeffRef(round(point.coeffRef(2) * divisionsPerUnit),
                                       round(point.coeffRef(1) * distanceToCylinder * divisionsPerUnit)) = 1;
        }
        return cylinderDivisions.sum();
    }
}
