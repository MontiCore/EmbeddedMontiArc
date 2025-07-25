/* (c) https://github.com/MontiCore/monticore */
#ifndef CNNPREDICTOR_TORCS_AGENT_TORCSAGENT_ACTOR
#define CNNPREDICTOR_TORCS_AGENT_TORCSAGENT_ACTOR

#include <mxnet/c_predict_api.h>

#include <cassert>
#include <string>
#include <vector>

#include <CNNBufferFile.h>

class CNNPredictor_torcs_agent_torcsAgent_actor_0{
public:
    const std::string json_file = "model/torcs.agent.network.TorcsActor/model_0_newest-symbol.json";
    const std::string param_file = "model/torcs.agent.network.TorcsActor/model_0_newest-0000.params";
    const std::vector<std::string> input_keys = {
        "data"
    };
    const std::vector<std::vector<mx_uint>> input_shapes = {{1, 29}};
    const bool use_gpu = false;

    PredictorHandle handle;

    explicit CNNPredictor_torcs_agent_torcsAgent_actor_0(){
        init(json_file, param_file, input_keys, input_shapes, use_gpu);
    }

    ~CNNPredictor_torcs_agent_torcsAgent_actor_0(){
        if(handle) MXPredFree(handle);
    }

    void predict(const std::vector<float> &state,
                 std::vector<float> &commands){
        MXPredSetInput(handle, "data", state.data(), static_cast<mx_uint>(state.size()));

        MXPredForward(handle);

        mx_uint output_index;
        mx_uint *shape = 0;
        mx_uint shape_len;
        size_t size;

        output_index = 0;
        MXPredGetOutputShape(handle, output_index, &shape, &shape_len);
        size = 1;
        for (mx_uint i = 0; i < shape_len; ++i) size *= shape[i];
        assert(size == commands.size());
        MXPredGetOutput(handle, 0, &(commands[0]), commands.size());

    }

    void init(const std::string &json_file,
              const std::string &param_file,
              const std::vector<std::string> &input_keys,
              const std::vector<std::vector<mx_uint>> &input_shapes,
              const bool &use_gpu){

        BufferFile json_data(json_file);
        BufferFile param_data(param_file);

        int dev_type = use_gpu ? 2 : 1;
        int dev_id = 0;

        if (json_data.GetLength() == 0 ||
            param_data.GetLength() == 0) {
            std::exit(-1);
        }

        const mx_uint num_input_nodes = input_keys.size();

        const char* input_keys_ptr[num_input_nodes];
        for(mx_uint i = 0; i < num_input_nodes; i++){
            input_keys_ptr[i] = input_keys[i].c_str();
        }

        mx_uint shape_data_size = 0;
        mx_uint input_shape_indptr[input_shapes.size() + 1];
        input_shape_indptr[0] = 0;
        for(mx_uint i = 0; i < input_shapes.size(); i++){
            shape_data_size += input_shapes[i].size();
            input_shape_indptr[i+1] = shape_data_size;
        }

        mx_uint input_shape_data[shape_data_size];
        mx_uint index = 0;
        for(mx_uint i = 0; i < input_shapes.size(); i++){
            for(mx_uint j = 0; j < input_shapes[i].size(); j++){
                input_shape_data[index] = input_shapes[i][j];
                index++;
            }
        }

        MXPredCreate(static_cast<const char*>(json_data.GetBuffer()),
                     static_cast<const char*>(param_data.GetBuffer()),
                     static_cast<size_t>(param_data.GetLength()),
                     dev_type,
                     dev_id,
                     num_input_nodes,
                     input_keys_ptr,
                     input_shape_indptr,
                     input_shape_data,
                     &handle);
        assert(handle);
    }
};

#endif // CNNPREDICTOR_TORCS_AGENT_TORCSAGENT_ACTOR
