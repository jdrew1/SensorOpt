#include "LiDAR.h"

void LiDAR::SetupCARLA(){
    CarlaAPI::RunPyScript("setupEnvironment");
}
void LiDAR::CloseCARLA(){
    CarlaAPI::RunPyScript("closeEnvironment");
}
void LiDAR::RunTest(){

}