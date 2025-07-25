import torch
import torch.nn as nn
import torch.nn.functional as F
from dgl.nn import GraphConv

class CNNNet_DGLNetwork(nn.Module):
    def __init__(self):
        super(CNNNet_DGLNetwork, self).__init__()

        self.graphconv3_ = GraphConv(in_feats=1433, out_feats=16, bias=True)
        # graphconv3_, output shape: {[1,2708,16]}

        self.layerOne_output_ = nn.ReLU()


        self.graphconv5_ = GraphConv(in_feats=16, out_feats=7, bias=True)
        # graphconv5_, output shape: {[1,2708,7]}

        self.predictions_ = nn.Identity()

        pass

    def forward(self , graph_ , features_ ):

        graphconv3_ = self.graphconv3_(graph_, features_)

        layerOne_output_ = self.layerOne_output_(graphconv3_)



        graphconv5_ = self.graphconv5_(graph_, layerOne_output_)

        predictions_ = self.predictions_(graphconv5_)

        return predictions_


