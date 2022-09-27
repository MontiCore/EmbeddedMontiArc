class TrainingConfiguration:
    def __init__(self):
        pass

    def getBatch_size(self):
        return 100
    def getEval_metric_typeValue(self):
        return "accuracy"
    def getNum_epoch(self):
        return 20
    def getLoss(self):
        return 'cross_entropy'
    def getoptimizerValue(self):
        return 'sgd'
    def getlearning_rate(self):
        return 0.001
    def getlearning_rate_decay(self):
        return 1.0
    def getstep_size(self):
        return 1000
    def getweight_decay(self):
        return 0.0
