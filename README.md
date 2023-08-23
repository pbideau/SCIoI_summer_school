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

# Tutorial 3: Lego Mindstorm Robots

## Connect

Connect to the following Wifi Network: "TP-Link_3F22_ActivePercpt".

Then use `ssh pi@<IP-ADDRESS>`, where you can find the IP address based on the list below.

Alan, 192.168.100
Ada, 192.168.0.101
Albert, 192.168.0.102
Joan, 192.168.0.103
Konrad, 192.168.0.104
Grace, 192.168.0.105
Hermann, 192.168.0.106
Kaethe, 192.168.0.107
Margaret, 192.168.0.108

