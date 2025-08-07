<#-- (c) https://github.com/MontiCore/monticore -->
<#-- So that the license is in the generated file: -->
/* (c) https://github.com/MontiCore/monticore */
#ifndef ${tc.fileNameWithoutEnding?upper_case}
#define ${tc.fileNameWithoutEnding?upper_case}

#include <tensorflow/core/public/session.h>
#include <tensorflow/core/protobuf/meta_graph.pb.h>

#include <cassert>
#include <string>
#include <vector>

class ${tc.fileNameWithoutEnding}_0{
public:
    const std::string graph_file = "model/${tc.componentName}/model_cpp_pred.meta";
    const std::string param_prefix = "model/${tc.componentName}/model_cpp_pred";

    tensorflow::Session* session;

    explicit ${tc.fileNameWithoutEnding}_0(){
        init(graph_file, param_prefix);
    }

    void predict(${tc.join(tc.architectureInputs, ", ", "const std::vector<float> &", "")},
                 ${tc.join(tc.architectureOutputs, ", ", "std::vector<float> &", "")}){
		

		std::vector<std::string> inputKeys = {<#list tc.architectureInputs as inputName>"${inputName}"<#if inputName?has_next>,</#if></#list>};
		
		
		std::vector<std::vector<float>> inputShapes = {<#list tc.architecture.inputs as input>{1,${tc.join(input.ioDeclaration.type.dimensions, ",")}}<#if input?has_next>,</#if></#list>};
		std::vector<tensorflow::TensorShape> inputShapesTensor = {<#list tc.architecture.inputs as input>tensorflow::TensorShape({1,${tc.join(input.ioDeclaration.type.dimensions, ",")}})<#if input?has_next>,</#if></#list>};
		
		std::vector<std::vector<float>> inputs =  {<#list tc.architectureInputs as inputName>${inputName}<#if inputName?has_next>,</#if></#list>};
		
		std::vector<std::pair<std::string, tensorflow::Tensor>> inputTensors; 
		
		for(int i=0; i<inputShapes.size(); i++){
			
			tensorflow::Tensor tempInputTensor = tensorflow::Tensor(tensorflow::DT_FLOAT, inputShapesTensor[i]);
			
			int numElems = inputShapes[i][0];
			for(int j=1; j<inputShapes[i].size(); j++){
				numElems *= inputShapes[i][j];
			}
			
			tensorflow::StringPiece tmpData = tempInputTensor.tensor_data();
			
			memcpy(const_cast<char*>(tmpData.data()), inputs[i].data(), numElems * sizeof(float));
			
			inputTensors.push_back({inputKeys[i], tempInputTensor});	
		}
		
        
        <#assign numOutputs=tc.architectureOutputs?size>
        std::vector<std::string> outputKeys = {<#list 0..(numOutputs-1) as i>"output_${i}"<#sep>,</#list>};
        
		std::vector<tensorflow::Tensor> outputTensor;
		

	
		TF_CHECK_OK(session->Run(inputTensors, outputKeys, {}, &outputTensor));
		
	
		
		std::vector<std::vector<float>*> outputs =  {<#list tc.architectureOutputs as outputName>&${outputName}<#if outputName?has_next>,</#if></#list>};
	
		for(int i=0; i < outputs.size(); i++){
                
            int flatOutSize = outputTensor[i].dim_size(0);
            for(int j=1; j < outputTensor[i].shape().dims(); j++){
                flatOutSize *= outputTensor[i].dim_size(j);
            }
            
			std::vector<float> currOutput; 
			for (int j=0; j < flatOutSize; j++){
				currOutput.push_back(outputTensor[i].flat<float>()(j));
			}
			*(outputs[i]) = currOutput;
		}
    }

    void init(const std::string &graph_file,
              const std::string &param_prefix){

    	session = tensorflow::NewSession(tensorflow::SessionOptions());
		if (session == nullptr) {
    		throw std::runtime_error("Could not create Tensorflow session.");
		}


		tensorflow::Status status;

		tensorflow::MetaGraphDef graph;
		status = tensorflow::ReadBinaryProto(tensorflow::Env::Default(), graph_file, &graph);
		if (!status.ok()) {
    		throw std::runtime_error("Error reading graph from " + graph_file + ": " + status.ToString());
		}


		status = session->Create(graph.graph_def());
		if (!status.ok()) {
    		throw std::runtime_error("Error adding graph to session: " + status.ToString());
		}

		tensorflow::Tensor pramPrefixTensor(tensorflow::DT_STRING, tensorflow::TensorShape());
		pramPrefixTensor.scalar<std::string>()() = param_prefix;

		status = session->Run({{ graph.saver_def().filename_tensor_name(), pramPrefixTensor },}, {}, {graph.saver_def().restore_op_name()}, nullptr);
		if (!status.ok()) {
    		throw std::runtime_error("Error loading weights from checkpoint:" + param_prefix + ": " + status.ToString());
		} 
    }
};

//For the tensorflow backend we need only one class for all streams, but the generated files require one for every stream, so dummy classes are generated here
<#assign numStreams=tc.architecture.streams?size>
<#if numStreams gte 2> 
<#list 1..(numStreams-1) as i>
class ${tc.fileNameWithoutEnding}_${i}{};  
</#list>
</#if>



#endif // ${tc.fileNameWithoutEnding?upper_case}
									 
