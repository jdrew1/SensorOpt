#ifndef NEURALPROJECT_PYTHONAPI_H
#define NEURALPROJECT_PYTHONAPI_H
#include <Python.h>
#include <stdio.h>
#include <string>

#include "settingsfile.h"
#include "logger.h"

namespace PythonAPI{
    void SetupPython(char ** args);

	PyObject* RunPyScript(std::string pyFunctName = SettingsFile::StringSetting("pyFunctName"),
                          std::string pythonArgs =  SettingsFile::StringSetting("pythonArgs"),
                          PyObject * inputObject = nullptr,
                          std::string pyScriptLoc = SettingsFile::StringSetting("pyScriptLoc"));

    std::string GetPyObjectFormat(PyObject* toParse = RunPyScript(), std::string buildup = "");
    void ExitPython();
}


#endif //NEURALPROJECT_PYTHONAPI_H