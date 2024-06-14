# 安装px4模拟器过程

这个系统是全新的ubuntu22.04，根据px4官方描述,虽然他们推荐使用Ubuntu22.04,但是对于拟真器（gazebo classic）来说Ubuntu22.04还没有经过较好的测试，这里推荐ubuntu20.04来使用拟真器。

下载源配置：http://mirrors.aliyun.com

设备需求：cpu需要在3核以上，因为需要一个作为代理客户端，一个作为px4模拟器，还有一个作为ROS2的控制端

1.安装了px4（以便使用PX4模拟器）

安装代码：

```jsx
cd
git clone [https://github.com/PX4/PX4-Autopilot.git](https://github.com/PX4/PX4-Autopilot.git) --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
cd PX4-Autopilot/
make px4_sitl
```

详情查看（https://docs.px4.io/main/zh/dev_setup/dev_env_linux_ubuntu.html &https://docs.px4.io/main/zh/dev_setup/building_px4.html）

1. 安装ROS2（Humble版本对应ubuntu22.04）（假如使用ubuntu20.04的话需要使用FOXY版本，虽然已经停止更新了）

```jsx
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL [https://raw.githubusercontent.com/ros/rosdistro/master/ros.key](https://raw.githubusercontent.com/ros/rosdistro/master/ros.key) -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] [http://packages.ros.org/ros2/ubuntu](http://packages.ros.org/ros2/ubuntu) $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt upgrade -y
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> .bashrc
```

3.安装python相关依赖

```jsx
pip install --user -U empy==3.3.4 pyros-genmsg setuptools
```

4.安装**Micro XRCE-DDS 代理(Agent)& 客户端(Client)**

重新开启一个端口

设置代理（agent）

代码如下：

```jsx
git clone [https://github.com/eProsima/Micro-XRCE-DDS-Agent.git](https://github.com/eProsima/Micro-XRCE-DDS-Agent.git)
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

此时安装流程已经结束

安装过程中出现的BUG：

- 没有按照顺序安装导致出现→make:*** No rule to make target ‘px4_sitl’ . Stop.

> 相应解决方案：按照顺序安装好，并且需要注意路径问题，不能进行线程多开的安装
>