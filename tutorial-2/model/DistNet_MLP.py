import torch
from torch import nn
from torch.utils.data import Dataset, DataLoader

INPUT_UNITS = 11 # Image hieght, image width, bounding box heigh, bounding box width, bounding box center,
                 # object class, aspect ratio, confidence, bounding box area and %area cover .
OUTPUT_UNITS = 1 # Distance

class MLP(nn.Module):
    """
    Multi-Layer Perceptron (MLP) neural network used for distance estimation.

    Args:
        hidden_units (int): Number of units in the hidden layers.
        n_layers (int): Number of hidden layers.
        act (str): Activation function to use. Default is 'relu'.
        act_trainable (bool): Whether to make the activation function trainable. Default is False.

    Attributes:
        net (torch.nn.Sequential): Sequential container for the MLP layers.

    """
    def __init__(self, hidden_units=256, n_layers=4, act='relu', act_trainable=False):
        super().__init__()
        layers = []
        for i in range(n_layers):

            if i == 0:
                l = nn.Linear(INPUT_UNITS, hidden_units)
            elif 0 < i < n_layers-1:
                l = nn.Linear(hidden_units, hidden_units)

            if act == 'relu':
                act_ = nn.ReLU(True)

            if i < n_layers-1:
                layers += [l, act_]
            else:
                layers += [nn.Linear(hidden_units, OUTPUT_UNITS), nn.Sigmoid()]

        self.net = nn.Sequential(*layers)

    def forward(self, x):
        return self.net(x)