import torch
import torch.nn as nn
import torch.nn.functional as F

class SupervisedEvaluation():

    def __init__(self, trained_model, test_loader):
        self._trained_model_ = trained_model
        self._test_loader_ = test_loader

    def execute(self):
        test_accuracy = []
        self._trained_model_.eval()
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
