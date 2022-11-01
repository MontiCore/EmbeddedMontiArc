class Supervised:

    def __init__(self):
        self.training_configuration = None
    def set_training_configuration(self,training_configuration):
        self.training_configuration =  training_configuration




    class Eval_metric_type:
        def __init__(self,outer):
            self.outer = outer
        def get_Eval_metric_type_Value(self):
            return self.outer.training_configuration.get_Eval_metric_type_Value()
        #common hyperparameters
        #value-specific hyperparameter
        def getaccuracy_ignore_labelaxis(self):
            return self.outer.training_configuration.getaccuracy_ignore_labelaxis()
        def getaccuracy_ignore_labelmetric_ignore_label(self):
            return self.outer.training_configuration.getaccuracy_ignore_labelmetric_ignore_label()
        def getbleuexclude(self):
            return self.outer.training_configuration.getbleuexclude()



    class Optimizer_type:
        def __init__(self,outer):
            self.outer = outer
        def get_Optimizer_type_Value(self):
            return self.outer.training_configuration.get_Optimizer_type_Value()
        #common hyperparameters
        def getoptimizer_typelearning_rate(self):
            return self.outer.training_configuration.getoptimizer_typelearning_rate()
        def getoptimizer_typelearning_rate_minimum(self):
            return self.outer.training_configuration.getoptimizer_typelearning_rate_minimum()
        def getoptimizer_typelearning_rate_decay(self):
            return self.outer.training_configuration.getoptimizer_typelearning_rate_decay()
        def getoptimizer_typeweight_decay(self):
            return self.outer.training_configuration.getoptimizer_typeweight_decay()
        def getoptimizer_typelearning_rate_policy(self):
            return self.training_configuration.optimizer_typelearning_rate_policy()
        def getoptimizer_typerescale_grad(self):
            return self.outer.training_configuration.getoptimizer_typerescale_grad()
        def getoptimizer_typeclip_gradient(self):
            return self.outer.training_configuration.getoptimizer_typeclip_gradient()
        def getoptimizer_typestep_size(self):
            return self.outer.training_configuration.getoptimizer_typestep_size()
        #value-specific hyperparameter
        def getsgdmomentum(self):
            return self.outer.training_configuration.getsgdmomentum()
        def getadambeta1(self):
            return self.outer.training_configuration.getadambeta1()
        def getadambeta2(self):
            return self.outer.training_configuration.getadambeta2()
        def getadamepsilon(self):
            return self.outer.training_configuration.getadamepsilon()
        def getadamwbeta1(self):
            return self.outer.training_configuration.getadamwbeta1()
        def getadamwbeta2(self):
            return self.outer.training_configuration.getadamwbeta2()
        def getadamwepsilon(self):
            return self.outer.training_configuration.getadamwepsilon()
        def getrmspropepsilon(self):
            return self.outer.training_configuration.getrmspropepsilon()
        def getrmspropgamma1(self):
            return self.outer.training_configuration.getrmspropgamma1()
        def getrmspropgamma2(self):
            return self.outer.training_configuration.getrmspropgamma2()
        def getrmspropcentered(self):
            return self.outer.training_configuration.getrmspropcentered()
        def getrmspropclip_weights(self):
            return self.outer.training_configuration.getrmspropclip_weights()
        def getrmsproprho(self):
            return self.outer.training_configuration.getrmsproprho()
        def getadagradepsilon(self):
            return self.outer.training_configuration.getadagradepsilon()
        def getnagmomentum(self):
            return self.outer.training_configuration.getnagmomentum()
        def getadadeltaepsilon(self):
            return self.outer.training_configuration.getadadeltaepsilon()
        def getadadeltarho(self):
            return self.outer.training_configuration.getadadeltarho()

    def getEval_metric_type(self):
        return self.Eval_metric_type(self)
    def getOptimizer_type(self):
        return self.Optimizer_type(self)
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
        return self.training_configuration.getcontext()