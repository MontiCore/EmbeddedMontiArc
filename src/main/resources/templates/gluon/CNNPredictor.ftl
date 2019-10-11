#ifndef ${tc.fileNameWithoutEnding?upper_case}
#define ${tc.fileNameWithoutEnding?upper_case}

#include <mxnet/c_predict_api.h>

#include <cassert>
#include <string>
#include <vector>

#include <CNNBufferFile.h>

<#list tc.architecture.networkInstructions as networkInstruction>
<#if networkInstruction.body.isTrainable()>
class ${tc.fileNameWithoutEnding}_${networkInstruction?index}{
public:
    const std::string json_file = "model/${tc.componentName}/model_${networkInstruction?index}_newest-symbol.json";
    const std::string param_file = "model/${tc.componentName}/model_${networkInstruction?index}_newest-0000.params";
    const std::vector<std::string> input_keys = {
<#if tc.getStreamInputNames(networkInstruction.body, false)?size == 1>
        "data"
<#else>
        <#list tc.getStreamInputNames(networkInstruction.body, false) as variable>"data${variable?index}"<#sep>, </#list>
</#if>
    };
    const std::vector<std::vector<mx_uint>> input_shapes = {<#list tc.getStreamInputDimensions(networkInstruction.body, true) as dimensions>{${tc.join(dimensions, ", ")}}<#sep>, </#list>};
    const bool use_gpu = false;

    PredictorHandle handle;

    explicit ${tc.fileNameWithoutEnding}_${networkInstruction?index}(){
        init(json_file, param_file, input_keys, input_shapes, use_gpu);
    }

    ~${tc.fileNameWithoutEnding}_${networkInstruction?index}(){
        if(handle) MXPredFree(handle);
    }

    void predict(${tc.join(tc.getStreamInputNames(networkInstruction.body, false), ", ", "const std::vector<float> &in_", "")},
                 ${tc.join(tc.getStreamOutputNames(networkInstruction.body), ", ", "std::vector<float> &out_", "")}){
<#list tc.getStreamInputNames(networkInstruction.body, false) as variable>
        MXPredSetInput(handle, input_keys[${variable?index}].c_str(), in_${variable}.data(), static_cast<mx_uint>(in_${variable}.size()));
</#list>

        MXPredForward(handle);

        mx_uint output_index;
        mx_uint *shape = 0;
        mx_uint shape_len;
        size_t size;

<#list tc.getStreamOutputNames(networkInstruction.body) as variable>
        output_index = ${variable?index?c};
        MXPredGetOutputShape(handle, output_index, &shape, &shape_len);
        size = 1;
        for (mx_uint i = 0; i < shape_len; ++i) size *= shape[i];
        assert(size == out_${variable}.size());
        MXPredGetOutput(handle, ${variable?index?c}, &(out_${variable}[0]), out_${variable}.size());

</#list>
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
</#if>
</#list>

#endif // ${tc.fileNameWithoutEnding?upper_case}
