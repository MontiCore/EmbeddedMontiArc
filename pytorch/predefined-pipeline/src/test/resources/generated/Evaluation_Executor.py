class Evaluation_Executor():
    def __init__(self, testData, trainedModel):
        self.testData = testData
        self.trainedModel = trainedModel

    def execute(self):
        accuracy = None
        return accuracy