#include "PythonAPI.h"

namespace PythonAPI{
    void SetupPython(char ** args){
        Py_Initialize();
        //format the arguments so that python can interpret them correctly:
        wchar_t **wargs = (wchar_t**)malloc(sizeof(wchar_t *));
        size_t * size = (size_t*)malloc(sizeof(size_t));
        *size = sizeof(**args);
        *wargs = Py_DecodeLocale(*args, size);
        //make sure python is able to see the files in the current directory
        PySys_SetArgv(1, wargs);
    }

    PyObject* RunPyScript(std::string pyFunctName, std::string pythonArgs, PyObject * inputObject, std::string pyScriptLoc){

        MyLogger::SaveToLog(("RunPyScript: Running Python Script: " +
                             pyFunctName + "(" + pythonArgs + ")").c_str(), MyLogger::Message);
        PyObject* pName = PyUnicode_FromString(pyScriptLoc.c_str());
        PyObject* pModule = PyImport_Import(pName);

        //if the python file was found, search for the function
        if(pModule){
            PyObject* pFunc = PyObject_GetAttrString(pModule, pyFunctName.c_str());
            //check the function can be called
            if (!PyCallable_Check(pFunc))
                MyLogger::SaveToLog(((std::string)"RunPyScript: Py function not callable: " + pyFunctName).c_str(), MyLogger::FATAL);
            //call the function as long as it exists
            if(pFunc){
                //get the return value and store into retValue
                PyObject* retValue;
                //build the format string based on the available inputs:
                //string only, just input the string (usually for compatability with calling from terminal)
                //object only, just input the object (usually parsed terminal arguments)
                //string & object, input tuple with the object then the string (parsed args and other input)
                std::string format = "";
                if (inputObject != nullptr)
                    format.append("O");
                if (!pythonArgs.empty())
                    format.append("s");

                switch (format.length()){
                    case 0:
                        retValue = PyObject_CallObject(pFunc, nullptr);
                        break;
                    case 1:
                        retValue = (inputObject == nullptr ?
                                        PyObject_CallFunction(pFunc, "s", pythonArgs.c_str())
                                      : PyObject_CallFunction(pFunc, "O", inputObject)
                                   );
                        break;
                    case 2:
                        retValue = PyObject_CallFunction(pFunc, "Os", inputObject, pythonArgs.c_str());
                        break;
                }
                if (retValue) return retValue;
            }
            else MyLogger::SaveToLog(("RunPyScript: Python Function not found: " + pyFunctName).c_str(), MyLogger::FATAL);
        }
        else MyLogger::SaveToLog(("RunPyScript: Python Module not found: " + pyScriptLoc).c_str(), MyLogger::FATAL);
        return nullptr;
    }

    std::string GetPyObjectFormat(PyObject* toParse, std::string buildup){
        MyLogger::SaveToLog("ParsePyObject: Interpreting Python Object", MyLogger::Message);
        if (toParse == nullptr) {
            MyLogger::SaveToLog("ParsePyObject: Python object is null", MyLogger::Error);
            return "";
        }
        std::string pyType = toParse->ob_type->tp_name;
        if (pyType != "list")
            return "";
        PyObject * firstPyObj = PySequence_GetItem(toParse,0);
        //printf(PyBytes_AsString(firstPyObj));
    }

    void ExitPython(){
        Py_Finalize();
    }
}