from SupervisedTrainer import SupervisedTrainer
from HDF5DataAccess import HDF5DataAccess
from SupervisedEvaluation import SupervisedEvaluation
from Schema2PythonAPI import Schema2PythonAPI
from CNNNet_mnist_mnistClassifier_net import CNNNet_mnist_mnistClassifier_net
from TrainingConfiguration import TrainingConfiguration

#imports for all implementations
class Pipeline_Executor():
    def __init__(self):
        pass

    def initialise_training_step(self, network=None,  schemaApi=None, trainData=None, model_dir=None, model_prefix=None):
        return SupervisedTrainer(network, schemaApi, trainData, model_dir, model_prefix)

    def initialize_data_access_step(self, datasource=None, schemaApi=None):
        return HDF5DataAccess(datasource, schemaApi)
        
    def initialise_evaluation_step(self, trainedModel=None, testData=None):
        return SupervisedEvaluation(trainedModel, testData)

    def execute(self):
        
        model_dir = "./model/mnist.LeNetNetwork/"
        datasource = "./data/"
        model_prefix = "model"
        
        network = CNNNet_mnist_mnistClassifier_net()
        schemaApi = Schema2PythonAPI()
        schemaApi.set_training_configuration(TrainingConfiguration())
        
        #step names come from instance block in reference model
        data_access_step = self.initialize_data_access_step(datasource, schemaApi)
        trainData, testData = data_access_step.execute()
        
        #arguments like trainData come from the connect block in the reference model
        training_step = self.initialise_training_step(network, trainData, schemaApi, model_dir, model_prefix)
        # names come from out ports  in reference model
        trainedModel, accuracy, loss = training_step.execute()
        
        evaluation_step = self.initialise_evaluation_step(trainedModel, testData)
        test_accuracy = evaluation_step.execute()
        print("Execution finished")
       
