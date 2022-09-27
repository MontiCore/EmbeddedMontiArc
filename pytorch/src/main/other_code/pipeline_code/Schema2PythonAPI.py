class Schema2PythonAPI:

    def __init__(self):
        self.training_configuration = None
    def set_training_configuration(self,training_configuration):
        self.training_configuration =  training_configuration


    class Eval_metric_type:
        def __init__(self,outer):
            self.outer = outer
        def getEval_metric_typeValue(self):
            return self.outer.training_configuration.getEval_metric_typeValue()
        def getaccuracy_ignore_labelaxis(self):
            return self.outer.training_configuration.getaccuracy_ignore_labelaxis()
        def getaccuracy_ignore_labelmetric_ignore_label(self):
            return self.outer.training_configuration.getaccuracy_ignore_labelmetric_ignore_label()
        def getbleuexclude(self):
            return self.outer.training_configuration.getbleuexclude()
    
    class optimizer:
        def __init__(self,outer):
            self.outer = outer
        def getoptimizerValue(self):
            return self.outer.training_configuration.getoptimizerValue()
        def getlearning_rate(self):
            return self.outer.training_configuration.getlearning_rate()
        def getlearning_rate_decay(self):
            return self.outer.training_configuration.getearning_rate_decay()
        def getstep_size(self):
            return self.outer.training_configuration.getstep_size()
        def getweight_decay(self):
            return self.outer.training_configuration.getweight_decay()

    def getoptimizer(self):
        return self.optimizer(self)
    def getBatch_size(self):
        return self.training_configuration.getBatch_size()
    def getNum_epoch(self):
        return self.training_configuration.getNum_epoch()
    def getEval_metric_type(self):
        return self.Eval_metric_type(self)
   

    def getcontext(self):
        return self.training_configuration.context()
    def getLoss(self):
        return self.training_configuration.getLoss()
        
    