import  HDF5DataAccess
import  MySupervisedTrainer
import  SupervisedEvaluation
import  CNNCreator_LeNet
import  Schema2PythonAPI
import  TrainingConfiguraiton
#imports for all implementations
class Pipeline_Executor():
    def __init__(self):
        pass

    # called component defined in pipeline configuration
    # arguments come from input ports and pipeline configuration
    def initialise_training_step(self, trainData=None, network=None,schemaApi=None):
        return MySupervisedTrainer(trainData= trainData, network= network, schemaApi= schemaApi)

    def initialise_data_access_step(self, datasource=None):
        return HDF5DataAccess(datasource= datasource)
    def initialise_evaluation_step(self, testData=None, schemaApi=None, trainedModel=None):
        return SupervisedEvaluation(testData, schemaApi, trainedModel)

    def execute(self):

        #step names come from instance block in reference model
        data_access_step = self.initialize_data_access_step(datasource="path.to.datasource.folder")
        trainData, testData = data_access_step.execute()
        network = CNNCreator_LeNet()
        schemaApi = Schema2PythonAPI()
        schemaApi.set_training_configuration(TrainingConfiguraiton())
        #arguments like trainData come from the connect block in the reference model
        training_step = self.initialise_training_step(trainData= trainData, network=network, schemaApi=schemaApi)
        # names come from out ports  in reference model
        trainedModel, accuracy, loss = training_step.execute()
        evaluation_step = self.initialise_evaluation_step(testData=testData, trainedModel=trainedModel, schemaApi=schemaApi)
        accuracy = evaluation_step.execute()
        print("Execution finished")
