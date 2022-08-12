#ifndef NEURALPROJECT_PYTHONAPI_H
#define NEURALPROJECT_PYTHONAPI_H
#include <Python.h>
#include <stdio.h>
#include <string>

#include "settingsfile.h"
#include "logger.h"

namespace CarlaAPI{
    void SetupPython(char ** args);

	PyObject* RunPyScript(std::string pyFunctName = SettingsFile::StringSetting("pyFunctName"),
                          std::string pythonArgs =  SettingsFile::StringSetting("pythonArgs"),
                          std::string pyScriptLoc = SettingsFile::StringSetting("pyScriptLoc"));

    void ParsePyObject(PyObject* toParse = RunPyScript());
    void ExitPython();
}


#endif //NEURALPROJECT_PYTHONAPI_H