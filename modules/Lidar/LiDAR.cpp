#include "LiDAR.h"

void LiDAR::SetupCARLA(){
    PyObject * args = PythonAPI::RunPyScript("parseArguments","-t False");
    PythonAPI::RunPyScript("setupEnvironment","",args);
    //PythonAPI::GetPyObjectFormat(PythonAPI::RunPyScript("trainLoop","0,0,0",args));
}
void LiDAR::RunTest(){
    //PyObject * coordinates = Py_BuildValue("s", &coords);
    //PyCapsule_SetName(coordinates, "coords");
    //PyObject * inputData = Py_BuildValue("ss", coordinates, inputObject);
    //PythonAPI::RunPyScript("trainLoop","",inputData);
}
void LiDAR::CloseCARLA(){
    PythonAPI::RunPyScript("closeEnvironment");
}