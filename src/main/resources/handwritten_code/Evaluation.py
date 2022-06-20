# (c) https://github.com/MontiCore/monticore
from abc import ABCMeta, abstractmethod

class AbstractEvaluation(metaclass=ABCMeta):
    @abstractmethod
    def evaluate(self):
        pass

class SupervisedEvaluation(AbstractEvaluation, metaclass=ABCMeta):

    def __init__(self, trained_model, train_config, test_loader):
        super().__init__()
        self._trained_model_ = trained_model
        self._test_loader_ = test_loader
        self._train_config_ = train_config

    def evaluate(self):
        self._trained_model_.eval()
        test_accuracy = []
        # since we're not training, we don't need to calculate the gradients for our outputs
        with torch.no_grad():
            for data in self._test_loader_:
                inputs, labels = data
                # calculate outputs by running images through the network
                outputs = self._trained_model_(inputs)
                # the class with the highest energy is what we choose as prediction
                _, predicted = torch.max(outputs.data, 1)
                test_accuracy.append(torch.tensor((torch.sum(predicted == labels).item() / len(predicted))))
            print(f'Accuracy:{(100 * (torch.tensor(test_accuracy)).mean().item()):.2f}%')
            accuracy = torch.stack(test_accuracy).mean().item()
        return accuracy