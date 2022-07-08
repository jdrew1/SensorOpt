#include "mainHeader.h"

int main(int argc, char **argv) {
    InitProgram(argv);
    //MNist::BasicMNISTPercept();

	CarlaAPI::RunPyScript();


    CleanProgram();
    return 0;
}
