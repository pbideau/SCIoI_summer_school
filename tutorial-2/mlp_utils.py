from pandas.io.common import file_exists
import ultralytics
import torch
import matplotlib.pyplot as plt
import numpy as np
import os
import glob
import cv2
import pandas as pd
import math

def get_robot_indices(robots, cam_robot,csv_path):

    df = pd.read_csv(csv_path, header=None, nrows=1)
    row = df.iloc[0].to_numpy()

    robot2index = {}
    for rbo in robots:
        bool_array = row == rbo
        first_idx = bool_array.argmax()
        robot2index[rbo] = first_idx

    return robot2index, robot2index[cam_robot]


def dist(robotX, robotY, camX, camY):
  return np.sqrt((float(robotX)-float(camX)) ** 2 + (float(robotY)-float(camY)) ** 2)


def load_data(model_YOLO, picture_files, df, data, robot2index, cam_robot_idx, verbose=False):
  for i, file in enumerate(picture_files):
    #row_value = file.split('/')[4]
    row_value = os.path.basename(file)  # cut path from image file name
    rows_matching_value = df[df.iloc[:, 0] == row_value]

    if rows_matching_value.empty:
        if verbose:
            print("Skipping, since no matching image name in csv")
        continue

    row_idx = rows_matching_value.index[0]

    if type(df.at[row_idx,2]) is not str and math.isnan(df.at[row_idx,2]):
        if verbose:
            print("Skipping, since no robots visable on image ")
        continue

    robot_list = df.at[row_idx,2].split()
    res = model_YOLO(file)

    bounding_boxes = res.xyxy[0][res.xyxy[0][:, 0].sort()[1]]

    if verbose:
        print("-----------NEW PICTURE-------------")
        print(f"In picture {row_idx} Number of robots: {bounding_boxes.shape[0]} List of robots: {robot_list}")

    if(bounding_boxes.shape[0] == len(robot_list)):
        for robot, box in zip(robot_list, bounding_boxes):
            camX = df.at[row_idx,cam_robot_idx+2]
            camY = df.at[row_idx,cam_robot_idx]
            if verbose:
               print(f"Cam X: {camX}, Cam Y: {camY}")
            robot_idx = robot2index[robot]
            robotX = df.at[row_idx,robot_idx + 2]
            robotY = df.at[row_idx,robot_idx]

            distance = dist(robotX, robotY, camX, camY)
            height = int(box[3]) - int(box[1])
            
            if verbose:
                print(f"Robot: {robot} Position: ({robotX},{robotY}) ")
                print(f"Height{height} Distance: {distance}")

            #values.setdefault(distance, []).append(int(box[3]) - int(box[1]))
            distance = torch.tensor([distance], dtype = torch.float32,)
            data.append(torch.cat((box, distance), 0))

def path2image(path: str, file_extension: str = "*.jpg"):
   """
    Retrieve a list of image file paths from the specified directory.

    Args:
        path (str): The directory path.
        file_extension (str, optional): The file extension pattern. Defaults to "*.jpg".

    Returns:
        List[str]: A list of image file paths.

    """
   file_pattern = os.path.join(path, file_extension)
   picture_files = glob.glob(file_pattern)
   return picture_files

def images2data(model, images, csv_path, verbose = False):
    """
    Extract object detection data from images using a detection model.

    Args:
        model (Yolo model): The object detection model.
        images (List[str]): A list of image file paths.

    Returns:
        List[torch.Tensor]: A list of tensors containing object detection data.

    """
    data = []
    
    # --- define robots
    ollie, grace, alan, kaethe, hermann = "ollie", "grace", "alan", "kaethe", "hermann"
    robots = [ollie, grace, alan, hermann, kaethe]
    cam_bot = ollie  # camera robot
    robot2index, cam_bot_idx = get_robot_indices(robots, cam_bot, csv_path)
    
    df = pd.read_csv(csv_path, header=None)
    load_data(model, images, df, data, robot2index, cam_bot_idx, verbose)

    return data