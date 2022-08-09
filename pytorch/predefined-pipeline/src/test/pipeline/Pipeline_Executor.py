import  HDF5DataAccess
import  SupervisedTrainer
import  CNNCreator_LeNet
import  Schema2PythonAPI
#imports for all implementations
class Pipeline_Executor():
    def __init__(self):
        pass

    # called component comes from configuration and parameters come from input ports and configuration
    def initialise_training_step(self, trainData=None, network=None,schemaApi=None):
        return SupervisedTrainer(trainData= trainData, network= network, schemaApi= schemaApi)

    def initialise_data_access_step(self, datasource=None):
        return HDF5DataAccess(datasource)

    # initialisation depends on execution
    def execute(self):
        data_access_step = self.initialize_data_access_step(datasource="generated value")
        trainData, testDate = data_access_step.execute()
        train_step = self.initialise_training_step( trainData= trainData, network= CNNCreator_LeNet(), schemaApi=Schema2PythonAPI())
        # names come from out ports  in reference model
        trainedModel, accuracy, loss = train_step.execute()
        print(f"Training finished {trainedModel, accuracy, loss}")
