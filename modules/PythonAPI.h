#ifndef NEURALPROJECT_PYTHONAPI_H
#define NEURALPROJECT_PYTHONAPI_H
#include <Python.h>
#include <stdio.h>
#include <string>

#include "settingsfile.h"

namespace CarlaAPI{
	FILE* filenamePyScript;
    static void SetupPython(){
	}
	static void RunPyScript(char **args){
		Py_Initialize();
		//format the arguments so that python can interpret them correctly:
		wchar_t **wargs = (wchar_t**)malloc(sizeof(wchar_t *));
		size_t * size = (size_t*)malloc(sizeof(size_t));
		*size = sizeof(**args);
		*wargs = Py_DecodeLocale(*args, size);
		//make sure python is able to see the files in the current directory
		PySys_SetArgv(1, wargs);

		PyObject* pName = PyUnicode_FromString("defaultPy");
		PyObject* pModule = PyImport_Import(pName);
        PyObject* pArgs = PyUnicode_FromString("--frames 10");

		if(pModule){
			PyObject* pFunc = PyObject_GetAttrString(pModule, "simple_return");
            if (PyCallable_Check(pFunc))
                printf("function not callable\n");
			if(pFunc && PyCallable_Check(pFunc)){
				PyObject* pValue = PyObject_CallObject(pFunc, pArgs);
                if (pValue) {
                    printf("Python Script returned a: ");
                    printf("%s",pValue->ob_type->tp_name);
                    printf("\nIt's value is: ");
                    printf("%ld",PyLong_AsLong(pValue));
                }
                else{
                    printf("No return value from python script\n");
                    //MyLogger::SaveToLog("No return value from Python script", MyLogger::Error);
                }
			}
			else{
				printf("ERROR: function getInteger()\n");
			}

		}
		else{
			printf("ERROR: Module not imported\n");
		}

		Py_Finalize();
	}
}


#endif //NEURALPROJECT_PYTHONAPI_H