import torch
from torch import nn

def train_step(model: torch.nn.Module,
               data_loader: torch.utils.data.DataLoader,
               loss_fn: torch.nn.Module,
               optimizer: torch.optim.Optimizer,
               device):
    """
    Perform a single training step for the model.

    Args:
        model (torch.nn.Module): The neural network model.
        data_loader (torch.utils.data.DataLoader): The data loader for training data.
        loss_fn (torch.nn.Module): The loss function.
        optimizer (torch.optim.Optimizer): The optimizer for updating model parameters.

    Returns:
        torch.Tensor: The average training loss for the step.

    """
    train_loss = 0

    for batch, (X, y) in enumerate(data_loader):
        X, y = X.to(device), y.to(device)
        y_pred = model(X)

        loss = loss_fn(y_pred, y)
        train_loss += loss

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

    train_loss /= len(data_loader)
    return train_loss

def validation_step(model: torch.nn.Module,
               data_loader: torch.utils.data.DataLoader,
               loss_fn: torch.nn.Module,
               device):
    """
    Perform a single validation step for the model.

    Args:
        model (torch.nn.Module): The neural network model.
        data_loader (torch.utils.data.DataLoader): The data loader for validation data.
        loss_fn (torch.nn.Module): The loss function.

    Returns:
        torch.Tensor: The average validation loss for the step.

    """
    with torch.no_grad():
      valid_loss = 0
      for batch, (X, y) in enumerate(data_loader):
          X, y = X.to(device), y.to(device)
          y_pred = model(X)

          loss = loss_fn(y_pred, y)
          valid_loss += loss

    valid_loss /= len(data_loader)
    return valid_loss
