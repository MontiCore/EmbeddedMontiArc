#ifndef CNNPREDICTOR_CIFARCLASSIFIERNETWORK
#define CNNPREDICTOR_CIFARCLASSIFIERNETWORK

#include "caffe2/core/common.h"
#include "caffe2/utils/proto_utils.h"
#include "caffe2/core/workspace.h"
#include "caffe2/core/tensor.h"
#include "caffe2/core/init.h"

// Define USE_GPU for GPU computation. Default is CPU computation.
//#define USE_GPU

#ifdef USE_GPU
#include "caffe2/core/context_gpu.h"
#endif

#include <string>
#include <iostream>
#include <map>

CAFFE2_DEFINE_string(init_net, "./model/CifarClassifierNetwork/init_net.pb", "The given path to the init protobuffer.");
CAFFE2_DEFINE_string(predict_net, "./model/CifarClassifierNetwork/predict_net.pb", "The given path to the predict protobuffer.");

using namespace caffe2;

class CNNPredictor_CifarClassifierNetwork{
    private:
        TensorCPU input;
        Workspace workSpace;
        NetDef initNet, predictNet;

    public:
        const std::vector<TIndex> input_shapes = {{1,3,32,32}};

        explicit CNNPredictor_CifarClassifierNetwork(){
            init(input_shapes);
        }

        ~CNNPredictor_CifarClassifierNetwork(){};

        void init(const std::vector<TIndex> &input_shapes){
            int n = 0;
            char **a[1];
            caffe2::GlobalInit(&n, a);

            if (!std::ifstream(FLAGS_init_net).good()) {
                std::cerr << "\nNetwork loading failure, init_net file '" << FLAGS_init_net << "' does not exist." << std::endl;
                exit(1);
            }

            if (!std::ifstream(FLAGS_predict_net).good()) {
                std::cerr << "\nNetwork loading failure, predict_net file '" << FLAGS_predict_net << "' does not exist." << std::endl;
                exit(1);
            }

            std::cout << "\nLoading network..." << std::endl;

            // Read protobuf
            CAFFE_ENFORCE(ReadProtoFromFile(FLAGS_init_net, &initNet));
            CAFFE_ENFORCE(ReadProtoFromFile(FLAGS_predict_net, &predictNet));

            // Set device type
            #ifdef USE_GPU
            predictNet.mutable_device_option()->set_device_type(CUDA);
            initNet.mutable_device_option()->set_device_type(CUDA);
            std::cout << "== GPU mode selected " << " ==" << std::endl;
            #else
            predictNet.mutable_device_option()->set_device_type(CPU);
            initNet.mutable_device_option()->set_device_type(CPU);

            for(int i = 0; i < predictNet.op_size(); ++i){
                predictNet.mutable_op(i)->mutable_device_option()->set_device_type(CPU);
            }
            for(int i = 0; i < initNet.op_size(); ++i){
                initNet.mutable_op(i)->mutable_device_option()->set_device_type(CPU);
            }
            std::cout << "== CPU mode selected " << " ==" << std::endl;
            #endif

            // Load network
            CAFFE_ENFORCE(workSpace.RunNetOnce(initNet));
            CAFFE_ENFORCE(workSpace.CreateNet(predictNet));
            std::cout << "== Network loaded " << " ==" << std::endl;

            input.Resize(input_shapes);
        }

        void predict(const std::vector<float> &data, std::vector<float> &softmax){
            //Note: ShareExternalPointer requires a float pointer.
            input.ShareExternalPointer((float *) data.data());

            // Get input blob
            #ifdef USE_GPU
            auto dataBlob = workSpace.GetBlob("data")->GetMutable<TensorCUDA>();
            #else
            auto dataBlob = workSpace.GetBlob("data")->GetMutable<TensorCPU>();
            #endif

            // Copy from input data
            dataBlob->CopyFrom(input);

            // Forward
            workSpace.RunNet(predictNet.name());

            // Get output blob
            #ifdef USE_GPU
            auto softmaxBlob = TensorCPU(workSpace.GetBlob("softmax")->Get<TensorCUDA>());
            #else
            auto softmaxBlob = workSpace.GetBlob("softmax")->Get<TensorCPU>();
            #endif
            softmax.assign(softmaxBlob.data<float>(),softmaxBlob.data<float>() + softmaxBlob.size());

            google::protobuf::ShutdownProtobufLibrary();
        }
};

#endif // CNNPREDICTOR_CIFARCLASSIFIERNETWORK