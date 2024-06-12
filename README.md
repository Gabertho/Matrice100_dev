# Matrice100_dev
Control repository for the DJI Matrice 100 UAV.

## pycontroller.py

Roscore:
```bash
roscore
```

Joystick:
```bash
rosrun joy joy_node __ns:=/drone
```

Simulator:
```bash
roslaunch drone_dev sim.launch ns:=/dji2 use_joy:=true
```

Trajectory:
```bash
roslaunch drone_dev trajectory.launch x:=30.0
```

Controller:
```bash
roslaunch drone_dev controller.launch
```

## Old Drone Dev

Roscore:
```bash
roscore
```

Joystick:
```bash
rosrun joy joy_node __ns:=/drone
```

Planner:
```bash
roslaunch drone_dev planner_node.launch
```

Controller:
```bash
roslaunch drone_dev matrice100_dev_node.launch
```

Simulator:
```bash
roslaunch drone_dev sim.launch ns:=/dji2
```

Rviz:
```bash
rviz -d lrs_ws/src/Matrice100_dev/test.rviz
```
