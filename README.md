# Initial Setup

## Software

We assume that you have Python 3.x with pip, and the ability to look at Jupyter notebooks. We recommend VS Code as an IDE.

## Virtual Environment (Optional)
Create a virtual environment
```
python3 -m venv venv
```

Now you need to activate the virtual environment
```
source venv/bin/activate
```

Next, install IPykernel which provides the Python kernel for Jupyter
```
pip install --user ipykernel
```

Add your virtual environment to Jupyter by typing
```
python -m ipykernel install --user --name=venv
```

# Tutorial 1: Dynamics, Controls, Planning, and Multi-Robot Collision Avoidance

All requirements are listed in the `requirements1.txt`, to install them simply run:

```
pip install -r requirements1.txt
```

Open `tutorial_1.ipynb` and follow the TODO's therein. 

# Tutorial 2: YOLOv5 for distance estimation

## Requirements

All requirements are listed in the `requirements2.txt`, to install them simply run:
```
pip install -r requirements2.txt
```

## Data

For training and evaluation we need to have pictures together with labels (robot distances). 
We recorded robot poses using a MoCap system. Robot poses are saved in csv format.
The required data can be downloaded [here](https://tubcloud.tu-berlin.de/s/Tk78deGXrgX497y).



