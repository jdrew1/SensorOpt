#ifndef NEURALPROJECT_TENSORFLOW_H
#define NEURALPROJECT_TENSORFLOW_H

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <tensorflow/c/c_api.h>
#include "settingsFile/settingsfile.h"
#include "logger/logger.h"


namespace TFNetwork{
    void InitNetwork();

    void LoadNetwork();
    void SaveNetwork();

    void LoadPoints();

    void Train();
    void Test();

}

#endif //NEURALPROJECT_TENSORFLOW_H