import Data_Access_Executor
import Training_Executor
import Evaluation_Executor

class Pipeline_Executor():
    def __init__(self):
        pass

    def execute(self):
        datasource = None
        network = None
        data_access_executor = Data_Access_Executor(datasource)
        data_access_executor.trainData, data_access_executor.testData = data_access_executor.execute()

        training_executor = Training_Executor(data_access_executor.trainData, network)
        training_executor.trainedModel, training_executor.accuracy, training_executor.loss = training_executor.execute()

        evaluation_executor = Evaluation_Executor(data_access_executor.testData, training_executor.trainedModel)
        evaluation_executor.accuracy = evaluation_executor.execute()
