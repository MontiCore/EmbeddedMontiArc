<#-- (c) https://github.com/MontiCore/monticore -->
<#-- So that the license is in the generated file: -->
# (c) https://github.com/MontiCore/monticore
import numpy as np

class ActionPolicyBuilder(object):
    def __init__(self):
        pass

    def build_by_params(self,
        method='epsgreedy',
        epsilon=0.5,
        min_epsilon=0.05,
        epsilon_decay_method='no',
        epsilon_decay=0.0,
        action_dim=None):

        if epsilon_decay_method == 'linear':
            decay = LinearDecay(eps_decay=epsilon_decay, min_eps=min_epsilon)
        else:
            decay = NoDecay()

        if method == 'epsgreedy':
            assert action_dim is not None
            assert len(action_dim) == 1
            return EpsilonGreedyActionPolicy(eps=epsilon,
                number_of_actions=action_dim[0], decay=decay)
        else:
            assert action_dim is not None
            assert len(action_dim) == 1
            return GreedyActionPolicy()

class EpsilonGreedyActionPolicy(object):
    def __init__(self, eps, number_of_actions, decay):
        self.eps = eps
        self.cur_eps = eps
        self.__number_of_actions = number_of_actions
        self.__decay_method = decay

    def select_action(self, values):
        do_exploration = (np.random.rand() < self.cur_eps)
        if do_exploration:
            action = np.random.randint(low=0, high=self.__number_of_actions)
        else:
            action = values.asnumpy().argmax()
        return action

    def decay(self):
        self.cur_eps = self.__decay_method.decay(self.cur_eps)


class GreedyActionPolicy(object):
    def __init__(self):
        pass

    def select_action(self, values):
        return values.asnumpy().argmax()

    def decay(self):
        pass


class NoDecay(object):
    def __init__(self):
        pass

    def decay(self, cur_eps):
        return cur_eps

class LinearDecay(object):
    def __init__(self, eps_decay, min_eps=0):
        self.eps_decay = eps_decay
        self.min_eps = min_eps

    def decay(self, cur_eps):
        return max(cur_eps - self.eps_decay, self.min_eps)
