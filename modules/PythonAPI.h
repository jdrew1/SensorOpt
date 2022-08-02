#ifndef NEURALPROJECT_PYTHONAPI_H
#define NEURALPROJECT_PYTHONAPI_H
#include <Python.h>
#include <stdio.h>
#include <string>

#include "settingsfile.h"

namespace CarlaAPI{
	FILE* filenamePyScript;
    static void SetupPython(char ** args){
        Py_Initialize();
        //format the arguments so that python can interpret them correctly:
        wchar_t **wargs = (wchar_t**)malloc(sizeof(wchar_t *));
        size_t * size = (size_t*)malloc(sizeof(size_t));
        *size = sizeof(**args);
        *wargs = Py_DecodeLocale(*args, size);
        //make sure python is able to see the files in the current directory
        PySys_SetArgv(1, wargs);
	}
	static void RunPyScript(){

		PyObject* pName = PyUnicode_FromString(SettingsFile::StringSetting("pyScriptLoc").c_str());
		PyObject* pModule = PyImport_Import(pName);
        const char * pyArgsString;
        SettingsFile::GetSetting("pythonArgs", &pyArgsString);

		if(pModule){
			PyObject* pFunc = PyObject_GetAttrString(pModule, "dotest");
            if (!PyCallable_Check(pFunc))
                printf("function not callable\n");
			if(pFunc && PyCallable_Check(pFunc)){
				PyObject* pValue = PyObject_CallFunction(pFunc, "s", pyArgsString);
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
	}
    static void ExitPython(){
        Py_Finalize();
    }
}


#endif //NEURALPROJECT_PYTHONAPI_H