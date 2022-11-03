#include "tensornetwork.h"

namespace TFNetwork{
    void InitNetwork(){
        TF_Graph* Graph = TF_NewGraph();
        TF_Status* Status = TF_NewStatus();

        TF_SessionOptions* SessionOpts = TF_NewSessionOptions();
        TF_Buffer* RunOpts = NULL;


    }

}