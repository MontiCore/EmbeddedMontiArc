/* (c) https://github.com/MontiCore/monticore */
#ifndef ${tc.fileNameWithoutEnding?upper_case}_0
#define ${tc.fileNameWithoutEnding?upper_case}_0
#include <cassert>
#include <string>
#include <vector>
#include <torch/script.h>
#include <torch/torch.h>

class ${tc.fileNameWithoutEnding}_0{
    public:
    const std::string pt_file = "./model/${tc.componentName}/model_cpp.pt";
    torch::jit::script::Module module;

    explicit ${tc.fileNameWithoutEnding}_0(){
        init(pt_file);
    }

    void init(const std::string &pt_file){

        try {
            // Deserialize the ScriptModule from a file using torch::jit::load().
            module = torch::jit::load(pt_file);
        }
        catch (const c10::Error& e) {
            throw std::runtime_error("Error loading the model\n");
        }
        std::cout << "Model loaded\n";
    }

    void predict(${tc.join(tc.architectureInputs, ", ", "const std::vector<float> &", "")},
                            ${tc.join(tc.architectureOutputs, ", ", "std::vector<float> &", "")}){

        at::Tensor inputTensors = torch::from_blob((float*)(<#list tc.architectureInputs as inputName>${inputName}<#if inputName?has_next>,</#if></#list>.data()), <#list tc.architecture.inputs as input>{1,${tc.join(input.ioDeclaration.type.dimensions, ",")}}<#if input?has_next>,</#if></#list>, at::TensorOptions());
        inputTensors = inputTensors.toType(at::kFloat);

        at::Tensor output = module.forward({inputTensors}).toTensor();
        std::vector<float> output_vector(output.data_ptr<float>(), output.data_ptr<float>() + output.numel());
        <#list tc.architectureOutputs as outputName>${outputName}<#if outputName?has_next>,</#if></#list> = output_vector;
    }
};
#endif // ${tc.fileNameWithoutEnding?upper_case}