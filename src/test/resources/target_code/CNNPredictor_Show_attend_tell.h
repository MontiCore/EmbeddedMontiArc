/* (c) https://github.com/MontiCore/monticore */
#ifndef CNNPREDICTOR_SHOW_ATTEND_TELL
#define CNNPREDICTOR_SHOW_ATTEND_TELL

#include <mxnet-cpp/MxNetCpp.h>

#include <cassert>
#include <string>
#include <vector>
    
#include <CNNModelLoader.h>
#include <CNNLAOptimizer_Show_attend_tell.h>

using namespace mxnet::cpp;    
    
class CNNPredictor_Show_attend_tell_0{
public:
    const std::string file_prefix = "model/Show_attend_tell/model_0_newest";
    
    //network
    const std::vector<std::string> network_input_keys = {
        "data"
    };
    const std::vector<std::vector<mx_uint>> network_input_shapes = {{1, 1}};
    std::vector<mx_uint> network_input_sizes;
    std::vector<std::vector<std::string>> network_arg_names;
    std::vector<Executor *> network_handles;
    
        
    //misc
    Context ctx = Context::cpu(); //Will be updated later in init according to use_gpu
    int dtype = 0; //use data type (float32=0 float64=1 ...)
 
                                                                                                           
    explicit CNNPredictor_Show_attend_tell_0(){
        init(file_prefix, network_input_keys, network_input_shapes);
    }

    ~CNNPredictor_Show_attend_tell_0(){
        for(Executor * handle : network_handles){
            delete handle;
        }
        MXNotifyShutdown();
    }

    void predict(const std::vector<float> &in_const1_,
                 std::vector<float> &out_target_0_){


        NDArray input_temp;
        input_temp = NDArray(network_input_shapes[0], ctx, false, dtype);
        input_temp.SyncCopyFromCPU(in_const1_.data(), network_input_sizes[0]);
        input_temp.CopyTo(&(network_handles[0]->arg_dict()[network_input_keys[0]]));
        NDArray::WaitAll();
    
        network_handles[0]->Forward(false);
        CheckMXNetError("Forward, predict, handle ind. 0");
    
        
        std::vector<NDArray> output = network_handles.back()->outputs;
        std::vector<mx_uint> curr_output_shape;
        size_t curr_output_size; 
        curr_output_shape = output[0].GetShape();
        curr_output_size = 1;
        for (mx_uint i : curr_output_shape) curr_output_size *= i;
        //Fix due to a bug in the in how the output arrays are initialized when there are multiple outputs
        assert((curr_output_size == out_target_0_.size()) || (curr_output_size == out_target_0_[0]));
        output[0].SyncCopyToCPU(&out_target_0_);
    
    }
    
    
    
    Executor* initExecutor(Symbol &sym,
                           std::map<std::string, NDArray> &param_map,
                           const std::vector<std::string> &exec_input_keys,
                           const std::vector<std::vector<mx_uint>> &exec_input_shapes){

        const mx_uint num_exec_input_nodes = exec_input_keys.size();
        for(mx_uint i = 0; i < num_exec_input_nodes; i++){
            param_map[exec_input_keys[i]] = NDArray(exec_input_shapes[i], ctx, false, dtype);
        }

        std::vector<NDArray> param_arrays;
        std::vector<NDArray> grad_array;
        std::vector<OpReqType> grad_reqs;
        std::vector<NDArray> aux_arrays;
        std::map< std::string, NDArray> aux_map;

        sym.InferExecutorArrays(ctx, &param_arrays, &grad_array, &grad_reqs,
                                    &aux_arrays, param_map, std::map<std::string, NDArray>(),
                                    std::map<std::string, OpReqType>(), aux_map);

        Executor *handle = new Executor(sym, ctx, param_arrays, grad_array, grad_reqs, aux_arrays);
        assert(handle);
        return handle;
    }

    std::vector<mx_uint> getSizesOfShapes(const std::vector<std::vector<mx_uint>> shapes){
        std::vector<mx_uint> sizes;
        for(std::vector<mx_uint> shape : shapes){
            mx_uint val = 1;
            for(mx_uint i: shape){
                val *= i;
            }
            sizes.push_back(val);
        }
        return sizes;
    }

    void CheckMXNetError(std::string loc){
        const char* err = MXGetLastError();
        if (err && err[0] != 0) {
            std::cout << "MXNet error at " << loc << err << std::endl;
            exit(-1);
        }
    }
    
    void init(const std::string &file_prefix,
              const std::vector<std::string> &network_input_keys,
              const std::vector<std::vector<mx_uint>> &network_input_shapes){

        CNNLAOptimizer_Show_attend_tell optimizer_creator = CNNLAOptimizer_Show_attend_tell();
    
        if(optimizer_creator.getContextName() == "gpu"){
            ctx = Context::gpu();
        }
            
        network_input_sizes = getSizesOfShapes(network_input_shapes);

        ModelLoader model_loader(file_prefix, 0, ctx);
    
        std::vector<Symbol> network_symbols = model_loader.GetNetworkSymbols();
        std::vector<std::map<std::string, NDArray>> network_param_maps;
        network_param_maps = model_loader.GetNetworkParamMaps();
    
        //Init handles
        std::map<std::string, std::vector<mx_uint>> in_shape_map;
        for(mx_uint i=0; i < network_input_keys.size(); i++){
            in_shape_map[network_input_keys[i]] = network_input_shapes[i];
        }
        std::vector<std::vector<mx_uint>> in_shapes;
        std::vector<std::vector<mx_uint>> aux_shapes;
        std::vector<std::vector<mx_uint>> out_shapes;
        network_symbols[0].InferShape(in_shape_map, &in_shapes, &aux_shapes, &out_shapes);
        network_handles.push_back(initExecutor(network_symbols[0], network_param_maps[0], network_input_keys, network_input_shapes));
    
    }
};
class CNNPredictor_Show_attend_tell_1{
public:
    const std::string file_prefix = "model/Show_attend_tell/model_1_newest";
    
    //network
    const std::vector<std::string> network_input_keys = {
        "data"
    };
    const std::vector<std::vector<mx_uint>> network_input_shapes = {{1, 3, 224, 224}};
    std::vector<mx_uint> network_input_sizes;
    std::vector<std::vector<std::string>> network_arg_names;
    std::vector<Executor *> network_handles;
    
        
    //misc
    Context ctx = Context::cpu(); //Will be updated later in init according to use_gpu
    int dtype = 0; //use data type (float32=0 float64=1 ...)
 
                                                                                                           
    explicit CNNPredictor_Show_attend_tell_1(){
        init(file_prefix, network_input_keys, network_input_shapes);
    }

    ~CNNPredictor_Show_attend_tell_1(){
        for(Executor * handle : network_handles){
            delete handle;
        }
        MXNotifyShutdown();
    }

    void predict(const std::vector<float> &in_images_,
                 std::vector<float> &out_features_output_){


        NDArray input_temp;
        input_temp = NDArray(network_input_shapes[0], ctx, false, dtype);
        input_temp.SyncCopyFromCPU(in_images_.data(), network_input_sizes[0]);
        input_temp.CopyTo(&(network_handles[0]->arg_dict()[network_input_keys[0]]));
        NDArray::WaitAll();
    
        network_handles[0]->Forward(false);
        CheckMXNetError("Forward, predict, handle ind. 0");
    
        
        std::vector<NDArray> output = network_handles.back()->outputs;
        std::vector<mx_uint> curr_output_shape;
        size_t curr_output_size; 
        curr_output_shape = output[0].GetShape();
        curr_output_size = 1;
        for (mx_uint i : curr_output_shape) curr_output_size *= i;
        //Fix due to a bug in the in how the output arrays are initialized when there are multiple outputs
        assert((curr_output_size == out_features_output_.size()) || (curr_output_size == out_features_output_[0]));
        output[0].SyncCopyToCPU(&out_features_output_);
    
    }
    
    
    
    Executor* initExecutor(Symbol &sym,
                           std::map<std::string, NDArray> &param_map,
                           const std::vector<std::string> &exec_input_keys,
                           const std::vector<std::vector<mx_uint>> &exec_input_shapes){

        const mx_uint num_exec_input_nodes = exec_input_keys.size();
        for(mx_uint i = 0; i < num_exec_input_nodes; i++){
            param_map[exec_input_keys[i]] = NDArray(exec_input_shapes[i], ctx, false, dtype);
        }

        std::vector<NDArray> param_arrays;
        std::vector<NDArray> grad_array;
        std::vector<OpReqType> grad_reqs;
        std::vector<NDArray> aux_arrays;
        std::map< std::string, NDArray> aux_map;

        sym.InferExecutorArrays(ctx, &param_arrays, &grad_array, &grad_reqs,
                                    &aux_arrays, param_map, std::map<std::string, NDArray>(),
                                    std::map<std::string, OpReqType>(), aux_map);

        Executor *handle = new Executor(sym, ctx, param_arrays, grad_array, grad_reqs, aux_arrays);
        assert(handle);
        return handle;
    }

    std::vector<mx_uint> getSizesOfShapes(const std::vector<std::vector<mx_uint>> shapes){
        std::vector<mx_uint> sizes;
        for(std::vector<mx_uint> shape : shapes){
            mx_uint val = 1;
            for(mx_uint i: shape){
                val *= i;
            }
            sizes.push_back(val);
        }
        return sizes;
    }

    void CheckMXNetError(std::string loc){
        const char* err = MXGetLastError();
        if (err && err[0] != 0) {
            std::cout << "MXNet error at " << loc << err << std::endl;
            exit(-1);
        }
    }
    
    void init(const std::string &file_prefix,
              const std::vector<std::string> &network_input_keys,
              const std::vector<std::vector<mx_uint>> &network_input_shapes){

        CNNLAOptimizer_Show_attend_tell optimizer_creator = CNNLAOptimizer_Show_attend_tell();
    
        if(optimizer_creator.getContextName() == "gpu"){
            ctx = Context::gpu();
        }
            
        network_input_sizes = getSizesOfShapes(network_input_shapes);

        ModelLoader model_loader(file_prefix, 0, ctx);
    
        std::vector<Symbol> network_symbols = model_loader.GetNetworkSymbols();
        std::vector<std::map<std::string, NDArray>> network_param_maps;
        network_param_maps = model_loader.GetNetworkParamMaps();
    
        //Init handles
        std::map<std::string, std::vector<mx_uint>> in_shape_map;
        for(mx_uint i=0; i < network_input_keys.size(); i++){
            in_shape_map[network_input_keys[i]] = network_input_shapes[i];
        }
        std::vector<std::vector<mx_uint>> in_shapes;
        std::vector<std::vector<mx_uint>> aux_shapes;
        std::vector<std::vector<mx_uint>> out_shapes;
        network_symbols[0].InferShape(in_shape_map, &in_shapes, &aux_shapes, &out_shapes);
        network_handles.push_back(initExecutor(network_symbols[0], network_param_maps[0], network_input_keys, network_input_shapes));
    
    }
};
class CNNPredictor_Show_attend_tell_2{
public:
    const std::string file_prefix = "model/Show_attend_tell/model_2_newest";
    
    //network
    const std::vector<std::string> network_input_keys = {
        "data0", "data1", "data2", "data3"
    };
    const std::vector<std::vector<mx_uint>> network_input_shapes = {{1, 256}, {1, 1, 512}, {1, 1}, {1, 1, 512}};
    std::vector<mx_uint> network_input_sizes;
    std::vector<std::vector<std::string>> network_arg_names;
    std::vector<Executor *> network_handles;
    
        
    //misc
    Context ctx = Context::cpu(); //Will be updated later in init according to use_gpu
    int dtype = 0; //use data type (float32=0 float64=1 ...)
 
                                                                                                           
    explicit CNNPredictor_Show_attend_tell_2(){
        init(file_prefix, network_input_keys, network_input_shapes);
    }

    ~CNNPredictor_Show_attend_tell_2(){
        for(Executor * handle : network_handles){
            delete handle;
        }
        MXNotifyShutdown();
    }

    void predict(const std::vector<float> &in_features_output_, const std::vector<float> &in_decoder_state_0_, const std::vector<float> &in_target_999999_, const std::vector<float> &in_decoder_state_1_,
                 std::vector<float> &out_target_1000000_, std::vector<float> &out_decoder_state_0_, std::vector<float> &out_decoder_state_1_, std::vector<float> &out_decoder_output_){


        NDArray input_temp;
        input_temp = NDArray(network_input_shapes[0], ctx, false, dtype);
        input_temp.SyncCopyFromCPU(in_features_output_.data(), network_input_sizes[0]);
        input_temp.CopyTo(&(network_handles[0]->arg_dict()[network_input_keys[0]]));
        input_temp = NDArray(network_input_shapes[1], ctx, false, dtype);
        input_temp.SyncCopyFromCPU(in_decoder_state_0_.data(), network_input_sizes[1]);
        input_temp.CopyTo(&(network_handles[0]->arg_dict()[network_input_keys[1]]));
        input_temp = NDArray(network_input_shapes[2], ctx, false, dtype);
        input_temp.SyncCopyFromCPU(in_target_999999_.data(), network_input_sizes[2]);
        input_temp.CopyTo(&(network_handles[0]->arg_dict()[network_input_keys[2]]));
        input_temp = NDArray(network_input_shapes[3], ctx, false, dtype);
        input_temp.SyncCopyFromCPU(in_decoder_state_1_.data(), network_input_sizes[3]);
        input_temp.CopyTo(&(network_handles[0]->arg_dict()[network_input_keys[3]]));
        NDArray::WaitAll();
    
        network_handles[0]->Forward(false);
        CheckMXNetError("Forward, predict, handle ind. 0");
    
        
        std::vector<NDArray> output = network_handles.back()->outputs;
        std::vector<mx_uint> curr_output_shape;
        size_t curr_output_size; 
        curr_output_shape = output[0].GetShape();
        curr_output_size = 1;
        for (mx_uint i : curr_output_shape) curr_output_size *= i;
        //Fix due to a bug in the in how the output arrays are initialized when there are multiple outputs
        assert((curr_output_size == out_target_1000000_.size()) || (curr_output_size == out_target_1000000_[0]));
        output[0].SyncCopyToCPU(&out_target_1000000_);
    
        curr_output_shape = output[1].GetShape();
        curr_output_size = 1;
        for (mx_uint i : curr_output_shape) curr_output_size *= i;
        //Fix due to a bug in the in how the output arrays are initialized when there are multiple outputs
        assert((curr_output_size == out_decoder_state_0_.size()) || (curr_output_size == out_decoder_state_0_[0]));
        output[1].SyncCopyToCPU(&out_decoder_state_0_);
    
        curr_output_shape = output[2].GetShape();
        curr_output_size = 1;
        for (mx_uint i : curr_output_shape) curr_output_size *= i;
        //Fix due to a bug in the in how the output arrays are initialized when there are multiple outputs
        assert((curr_output_size == out_decoder_state_1_.size()) || (curr_output_size == out_decoder_state_1_[0]));
        output[2].SyncCopyToCPU(&out_decoder_state_1_);
    
        curr_output_shape = output[3].GetShape();
        curr_output_size = 1;
        for (mx_uint i : curr_output_shape) curr_output_size *= i;
        //Fix due to a bug in the in how the output arrays are initialized when there are multiple outputs
        assert((curr_output_size == out_decoder_output_.size()) || (curr_output_size == out_decoder_output_[0]));
        output[3].SyncCopyToCPU(&out_decoder_output_);
    
    }
    
    
    
    Executor* initExecutor(Symbol &sym,
                           std::map<std::string, NDArray> &param_map,
                           const std::vector<std::string> &exec_input_keys,
                           const std::vector<std::vector<mx_uint>> &exec_input_shapes){

        const mx_uint num_exec_input_nodes = exec_input_keys.size();
        for(mx_uint i = 0; i < num_exec_input_nodes; i++){
            param_map[exec_input_keys[i]] = NDArray(exec_input_shapes[i], ctx, false, dtype);
        }

        std::vector<NDArray> param_arrays;
        std::vector<NDArray> grad_array;
        std::vector<OpReqType> grad_reqs;
        std::vector<NDArray> aux_arrays;
        std::map< std::string, NDArray> aux_map;

        sym.InferExecutorArrays(ctx, &param_arrays, &grad_array, &grad_reqs,
                                    &aux_arrays, param_map, std::map<std::string, NDArray>(),
                                    std::map<std::string, OpReqType>(), aux_map);

        Executor *handle = new Executor(sym, ctx, param_arrays, grad_array, grad_reqs, aux_arrays);
        assert(handle);
        return handle;
    }

    std::vector<mx_uint> getSizesOfShapes(const std::vector<std::vector<mx_uint>> shapes){
        std::vector<mx_uint> sizes;
        for(std::vector<mx_uint> shape : shapes){
            mx_uint val = 1;
            for(mx_uint i: shape){
                val *= i;
            }
            sizes.push_back(val);
        }
        return sizes;
    }

    void CheckMXNetError(std::string loc){
        const char* err = MXGetLastError();
        if (err && err[0] != 0) {
            std::cout << "MXNet error at " << loc << err << std::endl;
            exit(-1);
        }
    }
    
    void init(const std::string &file_prefix,
              const std::vector<std::string> &network_input_keys,
              const std::vector<std::vector<mx_uint>> &network_input_shapes){

        CNNLAOptimizer_Show_attend_tell optimizer_creator = CNNLAOptimizer_Show_attend_tell();
    
        if(optimizer_creator.getContextName() == "gpu"){
            ctx = Context::gpu();
        }
            
        network_input_sizes = getSizesOfShapes(network_input_shapes);

        ModelLoader model_loader(file_prefix, 0, ctx);
    
        std::vector<Symbol> network_symbols = model_loader.GetNetworkSymbols();
        std::vector<std::map<std::string, NDArray>> network_param_maps;
        network_param_maps = model_loader.GetNetworkParamMaps();
    
        //Init handles
        std::map<std::string, std::vector<mx_uint>> in_shape_map;
        for(mx_uint i=0; i < network_input_keys.size(); i++){
            in_shape_map[network_input_keys[i]] = network_input_shapes[i];
        }
        std::vector<std::vector<mx_uint>> in_shapes;
        std::vector<std::vector<mx_uint>> aux_shapes;
        std::vector<std::vector<mx_uint>> out_shapes;
        network_symbols[0].InferShape(in_shape_map, &in_shapes, &aux_shapes, &out_shapes);
        network_handles.push_back(initExecutor(network_symbols[0], network_param_maps[0], network_input_keys, network_input_shapes));
    
    }
};
#endif // CNNPREDICTOR_SHOW_ATTEND_TELL
