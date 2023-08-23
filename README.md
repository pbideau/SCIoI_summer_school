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

## 1. Connect

Connect to the following Wifi Network: "TP-Link_3F22_ActivePercpt".

We recommend using VS Code with the "Remote - SSH" extension. To connect:

1. Ctrl+Shift+P: "Remote-SSH: Add New SSH Host"
2. Type `ssh pi@<IP-ADDRESS>`, where you can find the IP address based on the list below.

Alan, 192.168.100
Ada, 192.168.0.101
Albert, 192.168.0.102
Joan, 192.168.0.103
Konrad, 192.168.0.104
Grace, 192.168.0.105
Hermann, 192.168.0.106
Kaethe, 192.168.0.107
Margaret, 192.168.0.108

3. Ctrl+Shift+P: "Remote-SSH: Connect to Host", select the host you added, and use `pi` as password.

You can now edit files like on your local machine. The integrated VS Code terminal is executing on the robot.

## 2. Camera Test

Execute

```
python3 camera.py
```

and inspect `test.jpg`.

## 3. Motor Test

Put the robot on the ground and execute

```
python3 motors.py
```

Note that the motor speed values are between -100 and 100. Play around with different values.

## 4. Perception

Execute

```
python3 perception.py
```

Follow the Visual Studio link to open meshcat in your browser and append `/static/` (including the trailing slash) to the URL.

## 5. Keyboard Control

In a second terminal (or a second person), use

```
python3 ssh_control.py
```

To control your robot with your keyboard.

## 6. State Estimate

In order to estimate the state of our robot, we use the robot dynamics as well with the known observed angular velocity of the wheels. The code is already prepared to keep track of the angular velocity of the wheels and calls a `state_estimate()` function in regular intervals. However, one still needs to use the robot dynamics to update the (belief) state.

Find the TODO in `state_estimation.py`, then test your code with different robot motions.

## 7. Robot Controller

## 8. 

