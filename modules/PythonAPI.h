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

	PyObject* RunPyScript(std::string pyScriptLoc = SettingsFile::StringSetting("pyScriptLoc"),
                          std::string pyFunctName = SettingsFile::StringSetting("pyFunctName"),
                          std::string pythonArgs =  SettingsFile::StringSetting("pythonArgs")){
        MyLogger::SaveToLog(("RunPyScript: Running Python Script: " +
                             pyFunctName + "(" + pythonArgs + ")").c_str(), MyLogger::Message);
		PyObject* pName = PyUnicode_FromString(pyScriptLoc.c_str());
		PyObject* pModule = PyImport_Import(pName);

		if(pModule){
			PyObject* pFunc = PyObject_GetAttrString(pModule, pyFunctName.c_str());
            if (!PyCallable_Check(pFunc))
                MyLogger::SaveToLog(((std::string)"RunPyScript: Py function not callable: " + pyFunctName).c_str(), MyLogger::FATAL);
			if(pFunc && PyCallable_Check(pFunc)){
				PyObject* pValue = PyObject_CallFunction(pFunc, "s", pythonArgs.c_str());
                if (pValue) return pValue;
                else MyLogger::SaveToLog("RunPyScript: No return value from Python script", MyLogger::Error);
			}
			else MyLogger::SaveToLog(("RunPyScript: Python Function not found: " + pyFunctName).c_str(), MyLogger::FATAL);
		}
		else MyLogger::SaveToLog(("RunPyScript: Python Module not found: " + pyScriptLoc).c_str(), MyLogger::FATAL);
        return nullptr;
	}

    static void ParsePyObject(PyObject* toParse = RunPyScript()){
        MyLogger::SaveToLog("ParsePyObject: Interpreting Python Object", MyLogger::Message);
        if (toParse == nullptr) {
            MyLogger::SaveToLog("ParsePyObject: Python object is null", MyLogger::Error);
            return;
        }
        std::string pyType = toParse->ob_type->tp_name;
        if (pyType != "list")
            return;
        PyObject * firstPyObj = PySequence_GetItem(toParse,0);
        //printf(PyBytes_AsString(firstPyObj));
    }




    static void ExitPython(){
        Py_Finalize();
    }
}


#endif //NEURALPROJECT_PYTHONAPI_H