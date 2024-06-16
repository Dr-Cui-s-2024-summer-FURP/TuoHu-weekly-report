# 运行px4过程

1.启动代理并设置以连接运行在模拟器上的 uXRCE-DDS客户端(Client)

```jsx
MicroXRCEAgent udp4 -p 8888
```

2.PX4 模拟器自动启动 uXRCE-DDS客户端，连接到本地主机上的 UDP 8888 端口

开启一个终端

跳转到PX4-Autopilot文件夹的位置，并输入代码：

```jsx
make px4_sitl gz_x500
```

3.配置相关ROS2的环境

新开一个终端

```jsx
mkdir -p ~/ws_sensor_combined/src/
cd ~/ws_sensor_combined/src/
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git
cd ..
source /opt/ros/humble/setup.bash
colcon build
```

4.默认一下ROS2的运行路径

```jsx
cd ~/ws_sensor_combined/
source /opt/ros/humble/setup.bash
```

5.开启ROS2

```jsx
source install/local_setup.bash
```

6.运行ROS2监听器（这只是一个测试样例）

```jsx
ros2 launch px4_ros_com sensor_combined_listener.launch.py
```

运行中遇到的bug：

px4模拟器连接时没有出现Gazebo，循环输出时间

解决方案：重启电脑