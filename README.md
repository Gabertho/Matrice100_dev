# Matrice100_dev
Control repository for the DJI Matrice 100 UAV.

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
rosrun drone_dev sim_node __ns:=/dji2
```

Rviz:
```bash
rviz -d lrs_ws/src/Matrice100_dev/test.rviz
```
