# Matrice100_dev
Control repository for the DJI Matrice 100 UAV.

## Start order

IMPORTANT: Start controller before trajectory

## Spline

Not yet working properly: Start with flag to trajectory launch: spline:=true

## Test flight

- Press green button, make sure it stabilize on altitude.  Change adjust_thrust argument to controller launch iof needed
- Press D-pad up to enable yaw control
- Press yellow button to start full trajectory flying
- Move goal positon and press start button to fly to a new goal

## Joystick

- A: Auto
- B: Manual
- X: Send one position at the time for a trajectory
- Y: Send whoöe trajectory in one message

- D-pad left: Yaw - 30 Degree in simulator
- D-pad right: Yaw + 30 Degree in simulator

## pycontroller.py

Roscore (backpack):
```bash
roscore
```

Joystick (backpack):
```bash
rosrun joy joy_node __ns:=/drone
```

Vicon (backpack):
```bash
roslaunch lrs_vicon mat2.launch
```

Simulator:
```bash
roslaunch drone_dev sim.launch ns:=/dji2 use_joy:=true
```

Trajectory:
```bash
roslaunch drone_dev trajectory.launch x:=30.0 y:=5.0
```

Controller:
```bash
roslaunch drone_dev controller.launch
```
or
```bash
roslaunch drone_dev controller.launch djisim:=true
```
or
```bash
roslaunch drone_dev controller.launch vicon:=true
```


Rviz:
```bash
rviz -d lrs_ws/src/Matrice100_dev/test.rviz
```

If using hardware simulator on mat2 or flying with GPS:

```bash
roslaunch lrs_launch sdk_nuc.launch m600:=false ns:=/dji2 use_joy:=false
```

```bash
rosrun lrs_dji djiconnect _rate:=50 _vicon:=false _publish_pose:=true _publish_world_position:=true __ns:=/dji2
```


If using hardware on mat2 and vicon for position:

```bash
rosrun lrs_dji djiconnect _rate:=50 _vicon:=true _publish_pose:=false _publish_world_position:=false __ns:=/dji2
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
