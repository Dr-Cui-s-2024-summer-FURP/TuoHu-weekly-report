# Implementing ego-planner based on XTDrone

Refer to https://www.yuque.com/xtdrone/manual_cn/3d_motion_planning

```jsx
cp -r  ~/XTDrone/motion_planning/3d/ego_planner ~/catkin_ws/src/
cd ~/catkin_ws/
catkin_make #or catkin build ego_planner

```

There may be a bug where the 'em' package is not detected. Simply download the lower version of the empy package:

```jsx
pip install empy==3.3.2

```

Start the simulation

ego_planner requires input of depth map + camera pose or point cloud. Here, the combination of depth map + camera pose is used for simulation. The depth map comes from realsense_camera (remember to replace iris_stereo_camera with iris_realsense_camera in indoor1.launch). The file location is in the launch folder under PX4_Firmware, and the camera pose is calculated by VINS-Fusion.

First, follow the VIO tutorial to open the simulator and let the drone hover, then close the keyboard control.

Then open another terminal

Convert the coordinate system direction of the camera pose

```jsx
cd ~/XTDrone/motion_planning/3d
python ego_transfer.py iris 0

```

Start rviz

```jsx
cd ~/XTDrone/motion_planning/3d
rviz -d ego_rviz.rviz

```

Start ego_planner

```jsx
roslaunch ego_planner single_uav.launch

```

The default condition is to fly towards a single point.

Use

```jsx
roslaunch ego_planner run_in_xtdrone.launch

```

The default condition is to fly according to four points.

File location: ~/catkin_ws/src/ego_planner/plan_manage/launch/run_in_xtdrone.launch

```
<arg name="point_num" value="1" />

<arg name="point0_x" value="$(arg target_x)" />
<arg name="point0_y" value="$(arg target_y)" />
<arg name="point0_z" value="$(arg target_z)" />

<arg name="point1_x" value="6.0" />
<arg name="point1_y" value="0.0" />
<arg name="point1_z" value="1.5" />

<arg name="point2_x" value="8.0" />
<arg name="point2_y" value="-8.0" />
<arg name="point2_z" value="1.5" />

<arg name="point3_x" value="0.0" />
<arg name="point3_y" value="-15.0" />
<arg name="point3_z" value="1.0" />

<arg name="point4_x" value="-15.0" />
<arg name="point4_y" value="0.0" />
<arg name="point4_z" value="1.0" />

```

By modifying the flight type, you can select 1, so you can set the destination through the 2D Nav Goal in RVIZ.

```
<!-- 1: use 2D Nav Goal to select goal  -->
<!-- 2: use global waypoints below  -->
<arg name="flight_type" value="2" />

```

The 2D target point can only be set on the ground, making the drone prone to losing control. A solution has not been found yet.

For convenience, a sh file to open the terminal has been set up.

```jsx
touch vio.sh
gedit vio.sh

```

This is to start the VIO simulation

```jsx
#!/bin/bash
source ~/.bashrc #set environment

gnome-terminal -- bash -c "roslaunch px4 indoor1.launch; exec bash"
sleep 5s
gnome-terminal -- bash -c "cd ~/catkin_ws; bash scripts/xtdrone_run_vio.sh; exec bash"
sleep 3s

gnome-terminal -- bash -c "cd ~/XTDrone/sensing/slam/vio; python vins_transfer.py iris 0; exec bash"
sleep 1s

gnome-terminal -- bash -c "cd ~/XTDrone/communication; python multirotor_communication.py iris 0; exec bash"
sleep 1s

gnome-terminal -- bash -c "cd ~/XTDrone/control/keyboard; python multirotor_keyboard_control.py iris 1 vel; exec bash"

```

This is to start the ego planner

```jsx
touch ego.sh
gedit ego.sh

```

```jsx
#!/bin/bash
source ~/.bashrc #set environment
gnome-terminal -- bash -c "cd ~/XTDrone/motion_planning/3d;python ego_transfer.py iris 0; exec bash"
sleep 1s
gnome-terminal -- bash -c "cd ~/XTDrone/motion_planning/3d;rviz -d ego_rviz.rviz; exec bash"
sleep 1s
gnome-terminal -- bash -c "roslaunch ego_planner single_uav.launch; exec bash"
sleep 1s

```

First, grant file permissions

```jsx
chmod +x vio.sh
chmod +x ego.sh

```

Then run in order

```jsx
./vio.sh

```

After all terminals pop up

Let the drone take off and hover (press the i key to increase the lift, then press the b key to enter offboard mode, and finally press the t key to enter arming state. Then press the i key again to bring up the adjusted speed panel. After rising to a certain height, press the s key to hover).

Close the control panel terminal

Run

```jsx
./ego.sh

```