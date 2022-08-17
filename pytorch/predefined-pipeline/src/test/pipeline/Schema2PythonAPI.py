# Generated from a Supervised-learning schema

class Supervised_Schema_API:

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

    def getEval_metric_type(self):
        return self.Eval_metric_type(self)
    def getBatch_size(self):
        return self.training_configuration.getBatch_size()
    def getNum_epoch(self):
        return self.training_configuration.getNum_epoch()
    def getCheckpoint_period(self):
        return self.training_configuration.getCheckpoint_period()
    def getPreprocessing_name(self):
        return self.training_configuration.getPreprocessing_name()
    def getLoad_pretrained(self):
        return self.training_configuration.getLoad_pretrained()
    def getLog_period(self):
        return self.training_configuration.getLog_period()
    def getShuffle_data(self):
        return self.training_configuration.getShuffle_data()

    def getcontext(self):
        return self.training_configuration.context()