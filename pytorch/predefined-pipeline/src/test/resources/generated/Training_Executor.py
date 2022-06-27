class Training_Executor():
    def __init__(self, trainData, network):
        self.trainData = trainData
        self.network = network

    def execute(self):
        trainedModel = None
        accuracy = None
        loss = None
        return trainedModel, accuracy, loss