# 基于XTDrone实现ego-planner

参考https://www.yuque.com/xtdrone/manual_cn/3d_motion_planning

```jsx
cp -r  ~/XTDrone/motion_planning/3d/ego_planner ~/catkin_ws/src/
cd ~/catkin_ws/
catkin_make #or catkin build ego_planner
```

其中可能出现‘em’包没有被检测到的bug，下载低版本的empy包即可：

```jsx
pip install empy==3.3.2
```

启动仿真

ego_planner需要输入深度图+相机位姿或是点云，这里以深度图+相机位姿的组合为例进行仿真，深度图来源于realsense_camera，(记得把indoor1.launch的iris_stereo_camera换成iris_realsense_camera)文件位置在PX4_Firmware下的launch文件夹里，相机位姿由VINS-Fusion计算得到。

首先按照vio教程中打开模拟器然后将飞机悬停，然后关闭键盘控制

然后另开终端

转换相机位姿的坐标系方向

```jsx
cd ~/XTDrone/motion_planning/3d
python ego_transfer.py iris 0
```

启动rviz

```jsx
cd ~/XTDrone/motion_planning/3d
rviz -d ego_rviz.rviz
```

启动ego_planner

```jsx
roslaunch ego_planner single_uav.launch 
```

默认条件是照着一个点飞

使用

```jsx
roslaunch ego_planner run_in_xtdrone.launch
```

默认条件是按照四个点位飞行

文件位置~/catkin_ws/src/ego_planner/plan_manage/launch/run_in_xtdrone.launch

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

通过修改飞行类型，可以选择1，从而可以通过RVIZ中的2D Nav Goal来设置目的地。

```
<!-- 1: use 2D Nav Goal to select goal  -->
<!-- 2: use global waypoints below  -->
<arg name="flight_type" value="2" />

```

这里2D的目标点设置好像只能设置在地面上，飞机特别容易失控，目前还没找到方案解决

目前为了方便，设置了一个可以打开终端的sh文件

```jsx
touch vio.sh
gedit vio.sh
```

这里是启动vio仿真

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

这里是启动ego

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

首先给予文件权限

```jsx
chmod +x vio.sh
chmod +x ego.sh
```

然后按顺序运行

```jsx
./vio.sh
```

等待终端全部跳出后

将无人机升空悬停（按i键提升向上的升力，然后按b键进入offboard模式，最后按t键进入arming状态，这时再按一下i键调出调好的速度面板，等待上升一段距离后按s键悬停）

将调控板终端关闭

运行

```jsx
./ego.sh
```

#implement