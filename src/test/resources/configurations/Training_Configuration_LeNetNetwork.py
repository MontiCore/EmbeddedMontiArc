

class Training_Configuration_LeNetNetwork:
    def __init__(self):
        pass


    def get_num_epoch(self):
        return 20


    def get_batch_size(self):
        return 64
    

    def get_context(self):
        return 'gpu'
    

    def get_eval_metric_value(self):
        return 'accuracy'



    def get_loss_value(self):
        return 'cross_entropy'



    def get_optimizer_value(self):
        return 'adam'
    

    def get_optimizer_learning_rate(self):
        return 0.001
    

    def get_optimizer_learning_rate_policy(self):
        return 'fixed'
    

    def get_optimizer_weight_decay(self):
        return 0.001
    

    def get_optimizer_epsilon(self):
        return 0.00000001
    

    def get_optimizer_beta1(self):
        return 0.9
    

    def get_optimizer_beta2(self):
        return 0.999

