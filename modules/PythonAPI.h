#ifndef NEURALPROJECT_PYTHONAPI_H
#define NEURALPROJECT_PYTHONAPI_H
#include <Python.h>
#include <stdio.h>
#include <string>

    static void TestABC(std::string message){
        Py_Initialize();
        PyRun_SimpleString("print('Hello World from Embedded Python!!!')");
        Py_Finalize();
    }

#endif //NEURALPROJECT_PYTHONAPI_H