<#-- (c) https://github.com/MontiCore/monticore -->
#ifndef ${tc.fileNameWithoutEnding?upper_case}
#define ${tc.fileNameWithoutEnding?upper_case}

#include <mxnet-cpp/MxNetCpp.h>

#include <cassert>
#include <string>
#include <vector>
    
#include <CNNModelLoader.h>
#include <CNNLAOptimizer_${tc.getFullArchitectureName()}.h>

using namespace mxnet::cpp;    
    
<#list tc.architecture.networkInstructions as networkInstruction>
class ${tc.fileNameWithoutEnding}_${networkInstruction?index}{
public:
    const std::string file_prefix = "model/${tc.componentName}/model_${networkInstruction?index}_newest";
    
    //network
    const std::vector<std::string> network_input_keys = {
<#if tc.getStreamInputNames(networkInstruction.body, true)?size == 1>
        "data"
<#else>
        <#list tc.getStreamInputNames(networkInstruction.body, true) as variable>"data${variable?index}"<#sep>, </#list>
</#if>
    };
    const std::vector<std::vector<mx_uint>> network_input_shapes = {<#list tc.getStreamInputDimensions(networkInstruction.body) as dimensions>{1, ${tc.join(tc.cutDimensions(dimensions), ", ")}}<#sep>, </#list>};
    std::vector<mx_uint> network_input_sizes;
    std::vector<std::vector<std::string>> network_arg_names;
    std::vector<Executor *> network_handles;
    
<#if networkInstruction.body.replaySubNetworks?has_content> 
    //replay_memory
    //replay_sub_nets
    const std::vector<std::string> sub_network_input_key = { //currently the same for all replay subnetworks
        "data"
    };
    //loss
    const std::vector<std::string> loss_input_keys = {
        "data0",
        "data1"
    };
                           
    mx_uint num_outputs = ${tc.getStreamOutputNames(networkInstruction.body, false)?size};
    const std::vector<std::vector<mx_uint>> output_shapes = {<#list tc.getStreamOutputDimensions(networkInstruction.body) as dimensions>{1, ${tc.join(tc.cutDimensions(dimensions), ", ")}}<#sep>, </#list>};
    std::vector<Executor *> loss_handles;
    
    //replay query nets
    const std::vector<std::string> replay_query_input_keys = {
        "data"
    };
    std::vector<Executor *> replay_query_handles;
    
    //replay memories
    std::vector<std::map<std::string, NDArray>> replay_memory;
    
    //parameters for local adapt
    std::vector<bool> use_local_adaption = {};
    std::vector<std::string> dist_measure = {};
    std::vector<mx_uint> replay_k = {};
    std::vector<mx_uint> gradient_steps = {};
    Optimizer *optimizerHandle;

    mx_uint num_subnets = ${networkInstruction.body.replaySubNetworks?size};
</#if>
        
    //misc
    Context ctx = Context::cpu(); //Will be updated later in init according to use_gpu
    int dtype = 0; //use data type (float32=0 float64=1 ...)
 
                                                                                                           
    explicit ${tc.fileNameWithoutEnding}_${networkInstruction?index}(){
        init(file_prefix, network_input_keys, network_input_shapes);
    }

    ~${tc.fileNameWithoutEnding}_${networkInstruction?index}(){
        for(Executor * handle : network_handles){
            delete handle;
        }
<#if networkInstruction.body.replaySubNetworks?has_content>
        for(Executor *handle : replay_query_handles){
                delete handle;
        }
        for(Executor * handle : loss_handles){
            delete handle;
        }
        delete optimizerHandle;
</#if>
        MXNotifyShutdown();
    }

    void predict(${tc.join(tc.getStreamInputNames(networkInstruction.body, false), ", ", "const std::vector<float> &in_", "")},
                 ${tc.join(tc.getStreamOutputNames(networkInstruction.body, false), ", ", "std::vector<float> &out_", "")}){

<#if networkInstruction.body.replaySubNetworks?has_content> 
        std::vector<std::vector<float>> network_input = {${tc.join(tc.getStreamInputNames(networkInstruction.body, false), ", ", "in_", "")}};

        for(mx_uint i=1; i < num_subnets; i++){
            if(use_local_adaption[i-1]){
                local_adapt(i, replay_query_handles[i-1], replay_memory[i-1], network_input, network_input_keys, network_input_shapes, network_input_sizes, loss_input_keys, gradient_steps[i-1], dist_measure[i-1], replay_k[i-1]);
            }
        }
</#if>
    
<#list tc.getStreamInputNames(networkInstruction.body, false) as variable>
        NDArray input_temp(network_input_shapes[${variable?index}], ctx, false, dtype);
        input_temp.SyncCopyFromCPU(in_${variable}.data(), network_input_sizes[${variable?index}]);
        input_temp.CopyTo(&(network_handles[0]->arg_dict()[network_input_keys[${variable?index}]]));
</#list>
        NDArray::WaitAll();
    
        network_handles[0]->Forward(false);
        CheckMXNetError("Forward, predict, handle ind. 0");
    
<#if networkInstruction.body.replaySubNetworks?has_content> 
        for(int i=1; i < network_handles.size(); i++){
            NDArray prev_output = network_handles[i-1]->outputs[0];
            prev_output.CopyTo(&(network_handles[i]->arg_dict()[sub_network_input_key[0]]));
            NDArray::WaitAll();

            network_handles[i]->Forward(false);
            CheckMXNetError("Forward, predict, handle ind. " + std::to_string(i));
        }
        NDArray::WaitAll();
</#if>        
        
        std::vector<NDArray> output = network_handles.back()->outputs;
        std::vector<mx_uint> curr_output_shape;
        size_t curr_output_size; 
<#list tc.getStreamOutputNames(networkInstruction.body, false) as output_name>    
        curr_output_shape = output[${output_name?index}].GetShape();
        curr_output_size = 1;
        for (mx_uint i : curr_output_shape) curr_output_size *= i;
        assert(curr_output_size == out_${output_name}.size());
        output[${output_name?index}].SyncCopyToCPU(&out_${output_name});
    
</#list>                                 
    }
    
<#if networkInstruction.body.replaySubNetworks?has_content>     
    //perform local adaption, train network on examples, only use updated on one inference (locally), don't save them
    void local_adapt(int net_start_ind,
                     Executor * query_handle,
                     std::map<std::string, NDArray> &memory,
                     const std::vector<std::vector<float>> &in_data_,
                     const std::vector<std::string> &in_keys,
                     const std::vector<std::vector<mx_uint>> &in_shapes,
                     std::vector<mx_uint> &in_sizes,
                     const std::vector<std::string> &loss_keys,
                     mx_uint gradient_steps,
                     std::string dist_measure,
                     mx_uint k){

        NDArray prev_output;

        for(size_t i=0; i < in_keys.size(); i++){
            NDArray input_temp(in_shapes[i], ctx, false, dtype);
            input_temp.SyncCopyFromCPU(in_data_[i].data(), in_sizes[i]);
            input_temp.CopyTo(&(network_handles[0]->arg_dict()[in_keys[i]]));
        }
        NDArray::WaitAll();

        for(size_t i=0; i < net_start_ind; i++){
            network_handles[i]->Forward(false);
            CheckMXNetError("Network forward, local_adapt, handle ind. " + std::to_string(i));
            prev_output = network_handles[i]->outputs[0];

            if(i+1 < net_start_ind){
                prev_output.CopyTo(&(network_handles[i+1]->arg_dict()[sub_network_input_key[0]]));
                NDArray::WaitAll();
            }
        }

        prev_output.CopyTo(&(query_handle->arg_dict()[replay_query_input_keys[0]]));
        NDArray::WaitAll();

        query_handle->Forward(false);
        CheckMXNetError("Query net forward, local_adapt, replay layer " + std::to_string(net_start_ind-1));
        NDArray query_output = query_handle->outputs[0];

        std::vector<NDArray> samples = pick_samples(query_output, memory, k, dist_measure);

        Operator slice("slice_axis");
        slice.SetParam("axis", 1);
        slice.SetInput("data", samples[1]);
        NDArray labels;

        for(mx_uint i=0; i < gradient_steps; i++){
            for(mx_uint j=0; j < k; j++){
                samples[0].Slice(j,j+1).CopyTo(&(network_handles[net_start_ind]->arg_dict()[sub_network_input_key[0]]));
                slice.SetParam("begin", j);
                slice.SetParam("end", j+1);
                slice.Invoke(labels);

                network_handles[net_start_ind]->Forward(true);
                CheckMXNetError("Network forward, local_adapt, handle ind. " + std::to_string(net_start_ind));
                for(int k=net_start_ind+1; k < network_handles.size(); k++){
                    prev_output = network_handles[k-1]->outputs[0];
                    prev_output.CopyTo(&(network_handles[k]->arg_dict()[sub_network_input_key[0]]));
                    NDArray::WaitAll();

                    network_handles[k]->Forward(true);
                    CheckMXNetError("Network forward, local_adapt, handle ind. " + std::to_string(k));
                }

                //May need adjustment to actually work for multiple outputs
                std::vector<NDArray> last_grads;
                for(size_t k=0; k < num_outputs; k++){
                    std::vector<NDArray> network_output = network_handles.back()->outputs;
                    network_output[k].CopyTo(&(loss_handles[k]->arg_dict()[loss_keys[0]]));
                    labels.Slice(k,k+1).CopyTo(&(loss_handles[k]->arg_dict()[loss_keys[1]]));
                    NDArray::WaitAll();

                    loss_handles[k]->Forward(true);
                    CheckMXNetError("Loss forward, local_adapt");
                    loss_handles[k]->Backward();
                    CheckMXNetError("Loss backward, local_adapt");

                    //last_grads.push_back(loss_handle->outputs[0]);
                    last_grads.push_back(loss_handles[k]->grad_dict()[loss_input_keys[0]]); //only the gradient for the prediction input not the label <- is this right, seems so??
                }

                //j has to be int not size_t/ mx_uint here
                for(int k=network_handles.size()-1; k >= net_start_ind; k--){
                    network_handles[k]->Backward(last_grads);
                    CheckMXNetError("Network backward, local_adapt, handle ind. " + std::to_string(k));
                    last_grads = {network_handles[k]->grad_dict()[sub_network_input_key[0]]};
                }

                for(size_t k=net_start_ind; k < network_arg_names.size(); ++k) {
                    for(size_t k=0; k < network_arg_names[k].size(); k++){
                        if (network_arg_names[k][k] == sub_network_input_key[0]) continue;
                        optimizerHandle->Update(k, network_handles[k]->arg_arrays[k], network_handles[k]->grad_arrays[k]);
                    }
                }
                NDArray::WaitAll();
            }
        }
    }
    
    NDArray innerProdDist(NDArray &vec1, NDArray &vec2){
        Operator dot("dot");
        dot.SetParam("transpose_b", true);

        NDArray ret;
        dot.SetInput("lhs", vec1);
        dot.SetInput("rhs", vec2);
        dot.Invoke(ret);

        return ret;
    }
    
    NDArray l2Norm(NDArray &vec1, NDArray &vec2){
        Operator br_diff("broadcast_sub");
        Operator l2_norm("norm");
        l2_norm.SetParam("axis", 1);

        NDArray diff;
        br_diff.SetInput("lhs", vec1);
        br_diff.SetInput("rhs", vec2);
        br_diff.Invoke(diff);

        NDArray ret;
        l2_norm.SetInput("data", diff);
        l2_norm.Invoke(ret);

        return ret;
    }

    std::vector<NDArray> pick_samples(NDArray query_output, std::map<std::string, NDArray> memory, mx_uint k, std::string dist_measure){
        Operator top_k("topk");
        top_k.SetParam("k", k);
        top_k.SetParam("ret_typ", "indices");
        top_k.SetParam("dtype", "float32");
        Operator take_values("take");
        Operator take_labels("take");
        take_labels.SetParam("axis", 1);

        NDArray dist;
        if(dist_measure == "l2"){
            dist = l2Norm(query_output, memory["keys"]);
        }else if(dist_measure == "inner_prod"){
            dist = innerProdDist(query_output, memory["keys"]);
        }else{
            throw std::invalid_argument("Provided value for dist_measure is not supported.");
        }

        NDArray indices;

        top_k.SetInput("data", dist);
        top_k.Invoke(indices);

        std::vector<NDArray> ret;

        /*
        Note: The both inputs have to be set in this order.
              If you would set "indices" first, it would use indices as "a",
              regardless of the name,m which should be predefined.
              Probably a bug in mxnet-cpp.
        */
        take_values.SetInput("a", memory["values"]);
        take_values.SetInput("indices", indices);
        ret.push_back(take_values.Invoke()[0]);

        take_labels.SetInput("a", memory["labels"]);
        take_labels.SetInput("indices", indices);
        ret.push_back(take_labels.Invoke()[0]);

        return ret;
    }
</#if>
    
    
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

        CNNLAOptimizer_${tc.getFullArchitectureName()} optimizer_creator = CNNLAOptimizer_${tc.getFullArchitectureName()}();
    
        if(optimizer_creator.getContextName() == "gpu"){
            ctx = Context::gpu();
        }
            
        network_input_sizes = getSizesOfShapes(network_input_shapes);

<#if networkInstruction.body.replaySubNetworks?has_content> 
        //Init Replay Memory prediction
${tc.include(networkInstruction.body, "PREDICTION_PARAMETER")}
    
        ModelLoader model_loader(file_prefix, num_subnets, ctx);
    
        std::vector<Symbol> network_symbols = model_loader.GetNetworkSymbols();
        std::vector<std::map<std::string, NDArray>> network_param_maps;
        network_param_maps = model_loader.GetNetworkParamMaps();
    
        std::vector<Symbol> replay_query_symbols = model_loader.GetQuerySymbols();
        std::vector<std::map<std::string, NDArray>> replay_query_param_maps;
        replay_query_param_maps = model_loader.GetQueryParamMaps();

        replay_memory = model_loader.GetReplayMemory();
        
        std::vector<std::vector<mx_uint>> label_shapes;
        for(mx_uint i=0; i < num_outputs; i++){
            label_shapes.push_back({1, 1}); //<--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        }
    
        Symbol loss = MakeLoss(model_loader.GetLoss());
        std::map<std::string, NDArray> loss_param_map = model_loader.GetLossParamMap();

        //Init handles
        std::vector<std::vector<mx_uint>> prev_out_shapes = network_input_shapes;
    
        std::vector<std::string> curr_input_keys;
        for(mx_uint i=0; i < network_symbols.size(); i++){
            network_arg_names.push_back(network_symbols[i].ListArguments());

            if(i == 0){
                curr_input_keys = network_input_keys;
            }else{
                curr_input_keys = {sub_network_input_key[0]};
            }
            
            std::map<std::string, std::vector<mx_uint>> in_shape_map;
            for(mx_uint i=0; i < curr_input_keys.size(); i++){
                in_shape_map[curr_input_keys[i]] = prev_out_shapes[i];
            }

            std::vector<std::vector<mx_uint>> in_shapes;
            std::vector<std::vector<mx_uint>> aux_shapes;
            std::vector<std::vector<mx_uint>> out_shapes;
            network_symbols[i].InferShape(in_shape_map, &in_shapes, &aux_shapes, &out_shapes);
            network_handles.push_back(initExecutor(network_symbols[i], network_param_maps[i], curr_input_keys, prev_out_shapes));
           
            if(i >= 1){
                std::map<std::string, std::vector<mx_uint>> query_in_shape_map;
                for(mx_uint j=0; j < replay_query_input_keys.size(); j++){
                    query_in_shape_map[replay_query_input_keys[j]] = prev_out_shapes[j];
                }
                replay_query_handles.push_back(initExecutor(replay_query_symbols[i-1], replay_query_param_maps[i-1], replay_query_input_keys, prev_out_shapes));
            }
            prev_out_shapes = {out_shapes[0]};
        }

        for(mx_uint i=0; i < num_outputs; i++){
            loss_handles.push_back(initExecutor(loss, loss_param_map, loss_input_keys, {output_shapes[i], label_shapes[i]}));
        }
    
        optimizerHandle = optimizer_creator.getOptimizer();
    
<#else>
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
    
</#if>
    }
};
</#list>
#endif // ${tc.fileNameWithoutEnding?upper_case}
