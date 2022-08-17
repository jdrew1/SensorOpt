#include "mainHeader.h"

int main(int argc, char **argv) {
    InitProgram(argv);

	LiDAR::SetupCARLA();
    LiDAR::RunTest();
    LiDAR::CloseCARLA();
    CleanProgram();
    return 0;
}
