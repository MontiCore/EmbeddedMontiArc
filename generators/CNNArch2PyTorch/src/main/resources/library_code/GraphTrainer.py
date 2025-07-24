import os
import errno
import shutil
from Utils import *
import torch
import torch.nn as nn
import torch.nn.functional as F
class GraphTrainer():

    def __init__(self, network, schemaApi, graph, model_dir = "", model_prefix = ""):
        super().__init__()
        self._network_ = network
        self._schemaApi_ = schemaApi
        self._model_dir_ = model_dir
        self._graph_ = graph
        self._model_prefix_ = model_prefix

        
    def execute(self):
        optimizer = translate_optim_name(list(self._network_.parameters()), self._schemaApi_.getOptimizer_type())
        best_val_acc = 0
        best_test_acc = 0
        if os.path.isdir(self._model_dir_):
            shutil.rmtree(self._model_dir_)
            
        try:
            os.makedirs(self._model_dir_)
        except OSError:
            if not os.path.isdir(self._model_dir_):
                raise
                
        features = self._graph_.ndata['feat']
        labels = self._graph_.ndata['label']
        train_mask = self._graph_.ndata['train_mask'].to(torch.bool)
        val_mask = self._graph_.ndata['val_mask'].to(torch.bool)
        file_object = open('train.log', 'w')
        
        for e in range(self._schemaApi_.getNum_epoch()):
            # Forward
            logits = self._network_(self._graph_, features)
            # Compute prediction
            pred = logits.argmax(1)
            # Compute loss
            loss = F.cross_entropy(logits[train_mask], labels[train_mask])

            # Compute accuracy on training
            train_acc = (pred[train_mask] == labels[train_mask]).float().mean()
            val_acc = (pred[val_mask] == labels[val_mask]).float().mean()

            # Save the best validation accuracy
            if best_val_acc < val_acc:
                best_val_acc = val_acc
            

            # Backward
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            
            #Save the model
            torch.save(self._network_, self._model_dir_ + 'model_dgl.pt')
            
            
            print('In epoch {}, loss: {:.3f}, val acc: {:.3f} (best {:.3f})'.format(
                e, loss, val_acc, best_val_acc))
            file_object.write('In epoch {}, loss: {:.3f}, val acc: {:.3f} (best {:.3f})\n'.format(
                e, loss, val_acc, best_val_acc))
            
        file_object.write('After {} epoch, loss: {:.3f}, accuracy: {:.3f} \n'.format(
                    self._schemaApi_.getNum_epoch(), loss, val_acc))
                    
        return self._network_ , val_acc , loss