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
		const wchar_t* pathname = Py_GetPath();
		const wchar_t* pathname3 = Py_GetExecPrefix();
		const wchar_t* pathname2 = Py_GetPrefix();
		const wchar_t* pathname4 = Py_GetProgramName();
		if (pathname && pathname2 && pathname3 &&pathname4)
			int i = 0;

		PyObject* pName = PyUnicode_FromString("defaultPy");
		PyObject* pModule = PyImport_Import(pName);

		if(pModule){
			PyObject* pFunc = PyObject_GetAttrString(pModule, "main");
			if(pFunc && PyCallable_Check(pFunc)){
				PyObject* pValue = PyObject_CallObject(pFunc, NULL);

				printf("C: getInteger() = %ld\n", PyLong_AsLong(pValue));
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