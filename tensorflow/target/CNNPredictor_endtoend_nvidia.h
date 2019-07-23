#ifndef CNNPREDICTOR_ENDTOEND_NVIDIA
#define CNNPREDICTOR_ENDTOEND_NVIDIA

#include <tensorflow/core/public/session.h>
#include <tensorflow/core/protobuf/meta_graph.pb.h>

#include <cassert>
#include <string>
#include <vector>

class CNNPredictor_endtoend_nvidia_0{
public:
    const std::string graph_file = "model/endtoend.Nvidia/model_cpp_pred.meta";
    const std::string param_prefix = "model/endtoend.Nvidia/model_cpp_pred";

    tensorflow::Session* session;

    explicit CNNPredictor_endtoend_nvidia_0(){
        init(graph_file, param_prefix);
    }

    //~CNNPredictor_endtoend_nvidia(){
    //}

    void predict(const std::vector<float> &data,
                 std::vector<float> &ergebnis){
		

		std::vector<std::string> inputKeys = {"data"};
		
		
		//TODO: Look closer at this
		std::vector<std::vector<float>> inputShapes = {{1,9,256,455}};
		std::vector<tensorflow::TensorShape> inputShapesTensor = {tensorflow::TensorShape({1,9,256,455})};
		
		//*see **
		std::vector<std::vector<float>> inputs =  {data};
		
		std::vector<std::pair<std::string, tensorflow::Tensor>> inputTensors; 
		
		for(int i=0; i<inputShapes.size(); i++){
			
			tensorflow::Tensor tempInputTensor = tensorflow::Tensor(tensorflow::DT_FLOAT, inputShapesTensor[i]);
			
			//probably more efficient way possible (preimplemented?)
			int numElems = inputShapes[i][0];
			for(int j=1; j<inputShapes[i].size(); j++){
				numElems *= inputShapes[i][j];
			}
			
			tensorflow::StringPiece tmpData = tempInputTensor.tensor_data();
			
			//**Here is an error related of the constness of the input passed to predict,see also definition of inputs at *, not compiling
			//Hope this works vor std::vectors, but should, only tried with opencv Mat
			memcpy(const_cast<char*>(tmpData.data()), inputs[i].data(), numElems * sizeof(float));
			
			inputTensors.push_back({inputKeys[i], tempInputTensor});	
		}
		
		
		//std::vector<std::string> outputKeys = {"ergebnis"};
        
        std::vector<std::string> outputKeys = {"output_0"};
        
		std::vector<tensorflow::Tensor> outputTensor;
		

	
		TF_CHECK_OK(session->Run(inputTensors, outputKeys, {}, &outputTensor));
		
	
		
		std::vector<std::vector<float>*> outputs =  {&ergebnis};
	
		for(int i=0; i < outputs.size(); i++){
			tensorflow::TTypes<float>::Matrix tempOut = outputTensor[i].matrix<float>();
			
			int outSize = outputTensor[i].dim_size(1); //only one dimensional outputs, may need to fix
	
			std::vector<float> currOutput; 
			for (int j=0; j < outSize; j++){
				currOutput.push_back(tempOut(j)); //can this be done like this or add directly to vectors passed to the function?	
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

#endif // CNNPREDICTOR_ENDTOEND_NVIDIA
