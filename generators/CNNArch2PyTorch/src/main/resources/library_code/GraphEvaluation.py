import torch
import torch.nn as nn
import torch.nn.functional as F

class GraphEvaluation():
    def __init__(self, trained_network, graph):
        self._trained_network_ = trained_network
        self._graph_ = graph
    
    def execute(self):
        self._trained_network_.eval()
        features = self._graph_.ndata['feat']
        labels = self._graph_.ndata['label']
        test_mask = self._graph_.ndata['test_mask'].to(torch.bool)
        with torch.no_grad():
            logits = self._trained_network_(self._graph_, features)
            logits = logits[test_mask]
            labels = labels[test_mask]
            _, indices = torch.max(logits, dim=1)
            correct = torch.sum(indices == labels)
            return correct.item() * 1.0 / len(labels)