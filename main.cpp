#include "mainHeader.h"

int main(int argc, char **argv) {
    InitProgram();
    //MNist::BasicMNISTPercept();

	CarlaAPI::RunPyScript(argv);

    return 0;
}
