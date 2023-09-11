import torch
from torch.utils.data import Dataset, DataLoader

MAX_DIST = 500 #Based on the Length of the MoCap Lab. Could be adjusted.
MAX_HEIGHT = 640 # This depends on the camera used
MAX_WIDTH = 640

class BBDataset(Dataset):
    """
    A PyTorch dataset for bounding box data.

    Args:
        data (list): The input data.
        im_height (int): The height of the image. Default is 640.
        im_width (int): The width of the image. Default is 640.
    """
    def __init__(self, data, im_height = MAX_HEIGHT, im_width = MAX_WIDTH):
        self.data = data
        self.im_height = torch.tensor(im_height, dtype = torch.float32) / MAX_HEIGHT
        self.im_width = torch.tensor(im_width, dtype = torch.float32) / MAX_WIDTH


    def __len__(self):
        return len(self.data)

    def __getitem__(self, idx):
        """
        Get the item at the given index.

        Args:
            idx (int): The index of the item.

        Returns:
            tuple: A tuple containing the input and distance.
        """
        
        BBH = (self.data[idx][3] - self.data[idx][1]) / MAX_HEIGHT

        input = torch.stack((self.im_height, BBH))

        distance = torch.tensor([self.data[idx][6] / MAX_DIST], dtype = torch.float32)

        return input, distance


def create_dataloader(train_data : list, test_data: list, batch_size: int = 1):
    """Creates training and testing DataLoaders.

    Takes in a training fata and testing data and turns
    them into PyTorch Datasets and then into PyTorch DataLoaders.

    Args:
        train_data: Tensor that holds the bounding box obtained from YOLO (res.xyxy[i], where i is the i-th object detected) and the distance to it. From the training data.
        test_data: Tensor that holds the bounding box obtained from YOLO (res.xyxy[i], where i is the i-th object detected). From the testing data.
        batch_size: Number of samples per batch in each of the DataLoaders.

    Returns:
        A tuple of (train_dataloader, test_dataloader).
    """
    train_dataset = BBDataset(train_data)
    test_dataset = BBDataset(test_data)
    train_dataloader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
    test_dataloader = DataLoader(test_dataset, batch_size=batch_size, shuffle=True)
    return train_dataloader, test_dataloader