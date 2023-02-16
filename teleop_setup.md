# Introduction
This document contains the steps necessary to install `ros`, `openpose`, the `azure kinect driver`, `ros openpose` and the `naoqi bridge`. This will enable you to do pose estimation on human bodies and copy the estimated pose to the robot. 

The general goal is to copy the pose of a human body to a robot. The pose of the human body is estimated through a computer vision model. Through this estimate, the joint angles of the joints on the human body can be derived, which can be translated to joint angles in the robot's joint space. We can then send these joint angles to robot to make the robot mimick us. 

There are several ways to do pose estimation, and this document provides instructions for different configurations. It is therefore not necessary to go through **every** step, and only the steps which are required for the configuration you want need to be followed. `Openpose` enables you to use RGB cameras (eg. webcams) to get started without an RGBD camera. However openpose is limited to 2D pose estimation unless you connect 2 cameras. Alternatively, an RGBD camera (such as the `Azure Kinect`) can do 3D pose estimation which is required to properly translate poses between the human and the robot. 

Ros is used as a middleware in any of the configurations, and the `Naoqi bridge` is necessary to interface with the Pepper robot from ros.

This software can only be used with linux and was tested on ubuntu 20 (however unless your hardware is very new, ubuntu 18 is probably easier to configure). Many install steps are dependent on the version you are using. 

Let's get started! I like to install these nice-to-haves but they are not necessary
```
sudo apt install terminator aptitude
sudo snap install vscode
```

# General requirements
These packages are necessary for multiple steps along the way and should be available on any linux distribution:
```
sudo apt install curl git libblas-dev liblapack-dev libatlas-base-dev python python3-pip
```

# Ros
Installing Robotic Operating System
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update
```

Different linux versions come with different ros versions, replace *noetic* with *melodic* if you are using ubuntu 18. See http://wiki.ros.org/Distributions for earlier versions. 

```
sudo apt install ros-noetic-desktop-full
```
Source environment correctly:

```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
```
python -m pip install PyYAML rospkg
```


# Azure kinect ros driver
This is necessary to get the Azure kinect RGBD camera working in ros. If you want to use other cameras with Openpose, you need other drivers. 
- [Realsense-ros](https://github.com/IntelRealSense/realsense-ros): For Intel RealSense Camera
- [iai_kinect2](https://github.com/code-iai/iai_kinect2): For Microsoft Kinect v2 Camera
- [zed-ros-wrapper](https://github.com/stereolabs/zed-ros-wrapper): For Stereolabs ZED2 Camera

Again, change 20.04 to 18.04 if on ubuntu 18.
```
curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo tee /etc/apt/trusted.gpg.d/microsoft.asc

sudo apt-add-repository https://packages.microsoft.com/ubuntu/20.04/prod
```

Somehow this package assumes an i386 architecture, modify `/etc/apt/sources.list`. At the bottom of the file, change this:

```
deb https://packages.microsoft.com/ubuntu/20.04/prod focal main
# deb-src https://packages.microsoft.com/ubuntu/20.04/prod focal main
```
to:
```
deb [arch=amd64] https://packages.microsoft.com/ubuntu/20.04/prod focal main
# deb-src [arch=amd64] https://packages.microsoft.com/ubuntu/20.04/prod focal main
```
---
If you are on ubuntu 18 you can install k4a, version 1.3 works out of the box. However, if you have an RTX 30XX graphics card you need version 1.4. You then also need to edit `$HOME/pepper_teleop/catkin_ws/src/Azure_Kinect_ROS_Driver/CMakeLists.txt` and [manually copy the sdk files](https://github.com/microsoft/Azure_Kinect_ROS_Driver/blob/melodic/docs/building.md#alternate-sdk-installation) to make sure it finds the right version 

```
sudo apt-get update
sudo apt install k4a-tools=1.3.*
sudo apt install libk4a1.3
sudo apt install libk4a1.3-dev
```

If you are on ubuntu 20, these packages are not available and it is easiest to just download them. Download the 1.4 versions `libk4a` and the 1.1 versions of `libk4abt` if you have an RTX 30XX card.
```
wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.3/libk4a1.3_1.3.0_amd64.deb
wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.3-dev/libk4a1.3-dev_1.3.0_amd64.deb
wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4abt1.0/libk4abt1.0_1.0.0_amd64.deb
wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4abt1.0-dev/libk4abt1.0-dev_1.0.0_amd64.deb
wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/k/k4a-tools/k4a-tools_1.3.0_amd64.deb

```
Install all above packages:
```
sudo dpkg -i libk4a1.3_1.3.0_amd64.deb
sudo dpkg -i libk4a1.3-dev_1.3.0_amd64.deb
sudo dpkg -i libk4abt1.0_1.0.0_amd64.deb
sudo dpkg -i libk4abt1.0-dev_1.0.0_amd64.deb
sudo dpkg -i libk4a1.3_1.3.0_amd64.deb
sudo dpkg -i k4a-tools_1.3.0_amd64.deb
```

The driver also requires `cuda`. There are installation instructions for `cuda` in the `openpose` section. If you are not going to also use `openpose`, you can install any version you like. 


---
## Azure kinect driver itself
```
sudo apt install ninja-build

mkdir $HOME/pepper_teleop/catkin_ws/src
cd $HOME/pepper_teleop/catkin_ws/src
git clone https://github.com/microsoft/Azure_Kinect_ROS_Driver.git

mkdir build && cd build
cmake .. -GNinja
ninja
sudo ninja install
```

From here, the Azure Kinect driver can perform 3D body tracking. You can lauch the body tracking with 

```
roslaunch azure_kinect_ros_driver driver_with_bodytracking.launch 
```





The points tracked in the Azure kinect tracking are the following:
![](https://learn.microsoft.com/en-us/azure/kinect-dk/media/concepts/joint-hierarchy.png)

The indices in the /body_tracking_data topic are as follows:


0. PELVIS 
1. SPINE_NAVEL
1. SPINE_CHEST
1. NECK  
1. CLAVICLE_LEFT
1. SHOULDER_LEFT
1. ELBOW_LEFT
1. WRIST_LEFT
1. HAND_LEFT
1. HANDTIP_LEFT
1. THUMB_LEFT
1. CLAVICLE_RIGHT
1. SHOULDER_RIGHT
1. ELBOW_RIGHT
1. WRIST_RIGHT
1. HAND_RIGHT
1. HANDTIP_RIGHT
1. THUMB_RIGHT
1. HIP_LEFT
1. KNEE_LEFT
1. ANKLE_LEFT
1. FOOT_LEFT
1. HIP_RIGHT
1. KNEE_RIGHT
1. ANKLE_RIGHT
1. FOOT_RIGHT
1. HEAD
1. NOSE
1. EYE_LEFT
1. EAR_LEFT
1. EYE_RIGHT
1. EAR_RIGHT

---
# Openpose 
Openpose can be used to estimate the pose of a human. It support multiple camera types, both RGB and RGBD. It can be build with cuda support to increase performance, but can also run on CPU if you do not have a graphics card. 

## Cuda
Openpose requires cuda. Some cuda versions might not work well, and you may need specific cuda versions to work with your specific graphics card. Cuda 11.7 supports almost all modern graphics cards and is tested with openpose so this version is recommended. 

Change `ubuntu2004` in the following with `ubuntu1804` if you are running ubuntu 18. 
```
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin

sudo mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600

wget https://developer.download.nvidia.com/compute/cuda/11.7.0/local_installers/cuda-repo-ubuntu2004-11-7-local_11.7.0-515.43.04-1_amd64.deb

sudo dpkg -i cuda-repo-ubuntu2004-11-7-local_11.7.0-515.43.04-1_amd64.deb

sudo cp /var/cuda-repo-ubuntu2004-11-7-local/cuda-*-keyring.gpg /usr/share/keyrings/

sudo apt-get update

sudo apt-get -y install cuda
```

## cuDnn
Like cuda, the specific version of cuDnn is important for the software to run correctly. Version 8.5 is tested and is hence recommended.

Change `ubuntu2004` in the following with `ubuntu1804` if you are running ubuntu 18. Change the cuda version if you are running an other cuda version than 11.7. 

```
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin 

sudo mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/3bf863cc.pub
sudo add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/ /"
sudo apt-get update

sudo apt-get install libcudnn8=8.5.0.*-1+cuda11.7
sudo apt-get install libcudnn8-dev=8.5.0.*-1+cuda11.7
```

## Openpose itself
Now we get to build openpose itself. 

```
sudo apt install libgflags-dev libgoogle-glog-dev

git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose.git

cd openpose

git checkout tags/v1.7.0

git submodule update --init --recursive --remote
```
If you do not have a graphics card, you can use openpose by building it for cpu only but the performance will be poor. You can do this by setting the `GPU_MODE` in the openpose `CMakeLists.txt` to `CPU_ONLY`. 
```
mkdir build/

cd build/

cmake ..

make -j`nproc`

sudo make install
```

If you get 'unsupported compute type' errors when building caffe as part of openpose, edit `/openpose/3rdparty/caffe/cmake/Cuda.cmake` and `$HOME/pepper_teleop/openpose/cmake/Cuda.cmake` to reflect the correct graphics card for your device (eg. `set(Caffe_known_gpu_archs "${AMPERE}")`)


# Ros openpose
An openpose to ros bridge, to use openpose recognized poses in ros.
*from https://github.com/ravijo/ros_openpose*

```
cd $HOME/pepper_teleop/catkin_ws/src
git clone https://github.com/ravijo/ros_openpose.git
cd $HOME/pepper_teleop/catkin_ws
catkin_make
```
---
There's a small bug in the code, which can sometimes cause the program to crash, to fix this change line 158 in `$HOME/pepper_teleop/catkin_ws/src/ros_openpose/scripts/visualizer.py` from
```
strip_id = idx / self.count_keypoints_one_finger
```
to 
```
strip_id = int(idx / self.count_keypoints_one_finger)
```
---

We can test ros_openpose with 
```
roslaunch ros_openpose run.launch camera:=azurekinect
```

If you want to render the hands as well add `--hand` to `openpose_args` and change `skeleton_hands` to `true` in `$HOME/pepper_teleop/catkin_ws/src/ros_openpose/launch/run.launch`

If you get out of memory errors, you can reduce the resolution of the openpose network with `--net-resolution -1x256` or smaller in `openpose_args` in `$HOME/pepper_teleop/catkin_ws/src/ros_openpose/launch/run.launch`

## openpose body points
![body points](https://cmu-perceptual-computing-lab.github.io/openpose/web/html/.github/media/keypoints_pose_25.png)

# Naoqi bridge
A bridge which is required to connect to the pepper robot from ros

Install the naoqi bridge with
```
sudo apt-get install ros-noetic-naoqi-bridge-msgs  ros-noetic-naoqi-libqicore
ros-noetic-naoqi-libqi 

cd $HOME/pepper_teleop/catkin_ws/src
git clone https://github.com/ros-naoqi/naoqi_driver.git
cd $HOME/pepper_teleop/catkin_ws/
catkin_make
```
Launch with
```
roslaunch naoqi_driver naoqi_driver.launch nao_ip:=192.168.1.61 roscore_ip:=127.0.0.1 network_interface:=eth0
```
The `roscore_ip` is probably correct, but the network interface might not be. This should be updated with whatever `ifconfig` says your network interface is called. This can be eth0/vpn1/wlp4s0/... The `nao_ip` should match the pepper's ip, which you can find out by pressing the button under the tablet once. 


## Your own code!

Use the following to create a new package with the dependencies you will need. 
```
catkin_create_pkg control_pepper geometry_msgs std_msgs rospy roscpp naoqi_bridge_msgs joy
```


You can publish [`naoqi_bridge_msgs/JointAnglesWithSpeed`](http://docs.ros.org/en/jade/api/naoqi_bridge_msgs/html/msg/JointAnglesWithSpeed.html) messages to the topic `/joint_angles` to command the joints. The `JointAnglesWithSpeed` consists of a `header`, `joint_names`, `joint_angles`, `speed` and `relative`

The `joint_names` are documented on http://doc.aldebaran.com/2-5/family/pepper_technical/joints_pep.html and can be the following:
  - HeadYaw
  - HeadPitch
  - LShoulderPitch
  - LShoulderRoll
  - LElbowYaw
  - LElbowRoll
  - LWristYaw
  - LHand
  - HipRoll
  - HipPitch
  - KneePitch
  - RShoulderPitch
  - RShoulderRoll
  - RElbowYaw
  - RElbowRoll
  - RWristYaw
  - RHand
  - WheelFL
  - WheelFR
  - WheelB

The `joint_angles` are values in **radians**. Naturally, the number of `joint_angles` should be equal to the number of `joint_names`, but they do not need to all be present. 

The `speed` is between 0 and 1, where 1 is the maximum speed

`relative` sets relative or absolute movement. 

