#include "mainHeader.h"

int main(int argc, char **argv) {
    InitProgram(argv);
    //MNist::BasicMNISTPercept();

	LiDAR::SetupCARLA();
    LiDAR::CloseCARLA();


    CleanProgram();
    return 0;
}
