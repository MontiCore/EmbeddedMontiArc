    '''
        possible values:
            sgd
            adam
            adamw
    '''

    def get_optimizer_value(self):
        return self.training_configuration.get_optimizer_value()

    #common hyperparameters
    def get_optimizer_learning_rate(self):
        return self.training_configuration.get_optimizer_learning_rate()
    def get_optimizer_learning_rate_policy(self):
        return self.training_configuration.get_optimizer_learning_rate_policy()

    #value-specific hyperparameters
    def get_optimizer_sgd_momentum(self):
        return self.training_configuration.get_optimizer_momentum()
    def get_optimizer_adam_beta1(self):
        return self.training_configuration.get_optimizer_beta1()
    def get_optimizer_adam_beta2(self):
        return self.training_configuration.get_optimizer_beta2()