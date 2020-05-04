<#-- (c) https://github.com/MontiCore/monticore -->
#ifndef ${tc.fileNameWithoutEnding?upper_case}
#define ${tc.fileNameWithoutEnding?upper_case}

#include <mxnet-cpp/MxNetCpp.h>

#include <cassert>
#include <string>
#include <vector>
    
#include <CNNModelLoader.h>

using namespace mxnet::cpp;    
    
<#list tc.architecture.networkInstructions as networkInstruction>
class ${tc.fileNameWithoutEnding}_${networkInstruction?index}{
public:
    const std::string file_prefix = "model/memoryLayerTest.Network/model_0_newest";
    //network
    const std::vector<std::string> network_input_keys = {
<#if tc.getStreamInputNames(networkInstruction.body, true)?size == 1>
        "data"
<#else>
        <#list tc.getStreamInputNames(networkInstruction.body, true) as variable>"data${variable?index}"<#sep>, </#list>
</#if>
    };
    const std::vector<std::string> sub_network_input_key = { //currently the same for all replay subnetworks
        "data"
    };
    const std::vector<std::vector<mx_uint>> network_input_shapes = {<#list tc.getStreamInputDimensions(networkInstruction.body) as dimensions>{1, ${tc.join(tc.cutDimensions(dimensions), ", ")}}<#sep>, </#list>};
    std::vector<mx_uint> network_input_sizes;
    std::vector<Executor *> network_handles;
    std::vector<std::vector<std::string>> network_arg_names;
    
    //loss
    const std::vector<std::string> loss_input_keys = {
        "data0",
        "data1"
    };
    const std::vector<std::vector<mx_uint>> loss_input_shapes = {{1, 10}, {1}}; //<--------------------------------------------
    Executor *loss_handle;
    
    //labels
    mx_uint num_outputs = ${tc.getStreamOutputNames(networkInstruction.body, false)?size};
    const std::vector<std::vector<mx_uint>> label_shapes = {{1}}; //<--------------------------------------------
    std::vector<mx_uint> label_sizes;                                                                                                                      
    
    //misc
    const bool use_gpu = false;
    Context ctx = Context::cpu(); //Will be updated later in init according to use_gpu
    //const bool perform_replay_adaption = true; //<-????????????????????????????????????????????????????
    mx_uint num_replay_layers = 4; //<-----------------------------------------------------------------------------------------------                                                                                                            
    int dtype = 0; //use data type (float32=0 float64=1 ...)
    
                                                                                                                
    explicit ${tc.fileNameWithoutEnding}_${networkInstruction?index}(){
        init(file_prefix, network_input_keys, network_input_shapes, use_gpu);
    }

    ~${tc.fileNameWithoutEnding}_${networkInstruction?index}(){
        for(Executor * handle : network_handles){
            delete handle;
        }

        if(loss_handle){
            delete loss_handle;
        }

        MXNotifyShutdown();
    }

    void predict(${tc.join(tc.getStreamInputNames(networkInstruction.body, false), ", ", "const std::vector<float> &in_", "")},
                 ${tc.join(tc.getStreamOutputNames(networkInstruction.body, false), ", ", "std::vector<float> &out_", "")}){

        if(num_replay_layers){ 
            //adapt to gven opimizer and optimizer params and potentially put into an init method
            Optimizer *optimizerHandle = OptimizerRegistry::Find("sgd"); //<---------------------------------
            //optimizerHandle->SetParam("rescale_grad", 1.0/batch_size); //<---------------------------------

            std::vector<std::vector<float>> replay_input_list;
            //for(mx_uint i=1; i < num_replay_layers+1; i++){
            //    replay_input_list = {in_data_};
            //    local_adapt(i, optimizerHandle, replay_input_list, network_input_keys, network_input_shapes, network_input_sizes, label, 1);
            //}

            //Temporary on full net:
            replay_input_list = {in_data_};
            const std::vector<std::vector<float>> label = {{1}}; //temporary
            local_adapt(0, optimizerHandle, replay_input_list, network_input_keys, network_input_shapes, network_input_sizes, label, 1);
        }
    
<#list tc.getStreamInputNames(networkInstruction.body, false) as variable>
        NDArray input_temp(network_input_shapes[${variable?index}], ctx, false, dtype);
        input_temp.SyncCopyFromCPU(in_${variable}.data(), network_input_sizes[${variable?index}]);
        input_temp.CopyTo(&(network_handles[0]->arg_dict()[network_input_keys[${variable?index}]]));
</#list>
        NDArray::WaitAll();
    
        network_handles[0]->Forward(false);
        CheckMXNetError("Forward, predict, handle ind. 0");
    
        for(int i=1; i < network_handles.size(); i++){
            NDArray prev_output = network_handles[i-1]->outputs[0];
            prev_output.CopyTo(&(network_handles[i]->arg_dict()[sub_network_input_key[0]]));
            NDArray::WaitAll();

            network_handles[i]->Forward(false);
            CheckMXNetError("Forward, predict, handle ind. " + std::to_string(i));
        }
        NDArray::WaitAll();
        
        std::vector<NDArray> output = network_handles.back()->outputs;
        std::vector<mx_uint> softmax_shape;
        size_t size; 
<#list tc.getStreamOutputNames(networkInstruction.body, false) as variable>    
        softmax_shape = output[${variable?index}].GetShape();
        size = 1;
        for (mx_uint i : softmax_shape) size *= i;
        assert(size == out_softmax_.size());

        /*
        Since the first element in the shape is the number of examples for which prediction is done
        And for us, at the moment this is always 1, we start from the second position
        */
        for(mx_uint i=0; i < softmax_shape[1]; i++){
            out_softmax_[i] = output[${variable?index}].At(0, i);
        }

        /*
        for(float i : out_softmax_){
            std::cout << i << std::endl;
        }
        */

        /*
        std::vector<NDArray> array = network_handles.back()->outputs;
        NDArray::WaitAll();

        for(NDArray i : array){
            std::cout << i << std::endl;
        }
        */                        
</#list>                                 
    }
    
    //perform local adaption, train network on examples, only use updated on one inference (locally), don't save them
    void local_adapt(int net_start_ind,
                     Optimizer *optimizerHandle,
                     const std::vector<std::vector<float>> &in_data_,
                     const std::vector<std::string> &in_keys,
                     const std::vector<std::vector<mx_uint>> &in_shapes,
                     std::vector<mx_uint> &in_sizes,
                     const std::vector<std::vector<float>> &labels,
                     mx_uint gradient_steps){

        for(size_t i=0; i < in_keys.size(); i++){
            NDArray input_temp(in_shapes[i], ctx, false, dtype);
            input_temp.SyncCopyFromCPU(in_data_[i].data(), in_sizes[i]);
            input_temp.CopyTo(&(network_handles[net_start_ind]->arg_dict()[in_keys[i]]));
        }
        NDArray::WaitAll();

        for(mx_uint i=0; i < gradient_steps; i++){
            network_handles[net_start_ind]->Forward(true);
            CheckMXNetError("Network forward, local_adapt, handle ind. 0");
            for(int j=net_start_ind+1; j < network_handles.size(); j++){
                NDArray prev_output = network_handles[j-1]->outputs[0];
                prev_output.CopyTo(&(network_handles[j]->arg_dict()[sub_network_input_key[0]]));
                NDArray::WaitAll();

                network_handles[j]->Forward(false);
                CheckMXNetError("Network forward, local_adapt, handle ind. " + std::to_string(j));
            }

            //May need adjustment to actually work for multiple outputs
            std::vector<NDArray> last_grads;
            for(size_t j=0; j < num_outputs; j++){
                std::vector<NDArray> network_output = network_handles.back()->outputs;
                network_output[j].CopyTo(&(loss_handle->arg_dict()[loss_input_keys[0]]));

                NDArray label_temp(label_shapes[j], ctx, false, dtype);
                label_temp.SyncCopyFromCPU(labels[j].data(), label_sizes[j]);
                label_temp.CopyTo(&(loss_handle->arg_dict()[loss_input_keys[1]]));

                loss_handle->Forward(true);
                CheckMXNetError("Loss forward, local_adapt");
                loss_handle->Backward();
                CheckMXNetError("Loss backward, local_adapt");

                //last_grads.push_back(loss_handle->outputs[0]);
                last_grads.push_back(loss_handle->grad_dict()[loss_input_keys[0]]); //only the gradient for the prediction input not the label <- is this right, seems so??
            }

            //j has to be int not size_t/ mx_uint here
            for(int j=network_handles.size()-1; j >= net_start_ind; j--){
                network_handles[j]->Backward(last_grads);
                CheckMXNetError("Network backward, local_adapt, handle ind. " + std::to_string(j));
                last_grads = {network_handles[j]->grad_dict()[sub_network_input_key[0]]};
            }

            for(size_t j=net_start_ind; j < network_arg_names.size(); ++j) {
                for(size_t k=0; k < network_arg_names[j].size(); k++){
                    if (network_arg_names[j][k] == sub_network_input_key[0]) continue;
                    optimizerHandle->Update(k, network_handles[j]->arg_arrays[k], network_handles[j]->grad_arrays[k]);
                }
            }
            NDArray::WaitAll();
        }
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
            //exit(-1);
        }
    }
    
    void init(const std::string &file_prefix,
              const std::vector<std::string> &network_input_keys,
              const std::vector<std::vector<mx_uint>> &network_input_shapes,
              const bool &use_gpu){

        if(use_gpu){
            ctx = Context::gpu();
        }
            
        network_input_sizes = getSizesOfShapes(network_input_shapes);
        label_sizes = getSizesOfShapes(label_shapes);
    
        ModelLoader model_loader(file_prefix, num_replay_layers, ctx);
    
        std::vector<Symbol> network_symbols = model_loader.GetNetworkSymbols();
        std::vector<std::map<std::string, NDArray>> network_param_maps;
        network_param_maps = model_loader.GetNetworkParamMaps();

        Symbol loss = MakeLoss(model_loader.GetLoss());
        std::map<std::string, NDArray> loss_param_map = model_loader.GetLossParamMap();

        std::map<std::string, std::vector<mx_uint>> in_shape_map;
        std::vector<std::vector<mx_uint>> prev_out_shapes = network_input_shapes;
    
        std::vector<std::string> curr_input_keys;
        for(mx_uint i=0; i < network_symbols.size(); i++){
            network_arg_names.push_back(network_symbols[i].ListArguments());

            if(i == 0){
                curr_input_keys = network_input_keys;
            }else{
                curr_input_keys = {sub_network_input_key[0]};
            }
            for(mx_uint i=0; i < curr_input_keys.size(); i++){
                in_shape_map[curr_input_keys[i]] = prev_out_shapes[i];
            }

            std::vector<std::vector<mx_uint>> in_shapes;
            std::vector<std::vector<mx_uint>> aux_shapes;
            std::vector<std::vector<mx_uint>> out_shapes;
            network_symbols[i].InferShape(in_shape_map, &in_shapes, &aux_shapes, &out_shapes);

            network_handles.push_back(initExecutor(network_symbols[i], network_param_maps[i], curr_input_keys, prev_out_shapes));
            prev_out_shapes = {out_shapes[0]};
        }
        loss_handle = initExecutor(loss, loss_param_map, loss_input_keys, loss_input_shapes);
    }
};
</#list>

#endif // ${tc.fileNameWithoutEnding?upper_case}
