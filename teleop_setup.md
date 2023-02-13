# Introduction

## general requirements

nice to haves
```
sudo apt install terminator aptitude
```


```
sudo apt install git libblas-dev liblapack-dev libatlas-base-dev
```

## ros
*Different linux versions come with different ros versions, replace* noetic *with other other versions as appropriate*

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt install curl # if you haven't already installed curl

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update

sudo apt install ros-noetic-desktop-full
```
Source environment correctly:

```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```



## openpose 

*prerequisites*

### cuda

```
wget https://developer.download.nvidia.com/compute/cuda/12.0.1/local_installers/cuda_12.0.1_525.85.12_linux.run
sudo sh cuda_12.0.1_525.85.12_linux.run

nvcc --version # to check

```

cuDnn
```
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-ubuntu1804.pin 

sudo mv cuda-ubuntu1804.pin /etc/apt/preferences.d/cuda-repository-pin-600
sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/3bf863cc.pub
sudo add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/ /"
sudo apt-get update

sudo apt-get install libcudnn8=8.8.0.*-1+cuda12.0
sudo apt-get install libcudnn8-dev=8.8.0.*-1+cuda12.0
```

*other*
```
sudo apt install libgflags-dev libgoogle-glog-dev
```


```
git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose.git

cd openpose

git checkout tags/v1.7.0

git submodule update --init --recursive --remote

mkdir build/

cd build/

cmake ..
make -j`nproc`
sudo make install

```


## ros openpose
*from https://github.com/ravijo/ros_openpose*

```
```

```
```
# Introduction

## general requirements

```
sudo apt install git libblas-dev liblapack-dev libatlas-base-dev
```

## ros
*Different linux versions come with different ros versions, replace* melodic *with other other versions as appropriate*

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt install curl # if you haven't already installed curl

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update

sudo apt install ros-melodic-desktop-full
```
Source environment correctly:

```
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```



## openpose 

*prerequisites*

### cuda

```
<!-- wget https://developer.download.nvidia.com/compute/cuda/12.0.1/local_installers/cuda_12.0.1_525.85.12_linux.run
sudo sh cuda_12.0.1_525.85.12_linux.run

nvcc --version # to check -->



wget -c "https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin"

sudo mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600

wget "https://developer.download.nvidia.com/compute/cuda/11.0.3/local_installers/cuda-repo-ubuntu2004-11-0-local_11.0.3-450.51.06-1_amd64.deb"

sudo dpkg -i cuda-repo-ubuntu2004-11-0-local_11.0.3-450.51.06-1_amd64.deb

sudo apt-key add /var/cuda-repo-ubuntu2004-11-0-local/7fa2af80.pub

sudo apt-get update

sudo apt-get -y install cuda


```

cuDnn
```
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-ubuntu1804.pin 

sudo mv cuda-ubuntu1804.pin /etc/apt/preferences.d/cuda-repository-pin-600
sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/3bf863cc.pub
sudo add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/ /"
sudo apt-get update

sudo apt-get install libcudnn8=8.8.0.*-1+cuda11.8
sudo apt-get install libcudnn8-dev=8.8.0.*-1+cuda11.8
```

*other*
```
sudo apt install libgflags-dev libgoogle-glog-dev
```


```
git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose.git

cd openpose

git checkout tags/v1.7.0

git submodule update --init --recursive --remote

mkdir build/

cd build/

cmake ..
make -j`nproc`
sudo make install

```

If you get 'unsupported compute type' errors when building caffe as part of openpose, edit ```$HOME/pepper_teleop/openpose/3rdparty/caffe/cmake/Cuda.cmake``` and ```$HOME/pepper_teleop/openpose/cmake/Cuda.cmake``` to reflect the correct graphics card for your device (eg. ```set(Caffe_known_gpu_archs "${AMPERE}")```)


## azure kinect ros driver

```
curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo tee /etc/apt/trusted.gpg.d/microsoft.asc

sudo apt-add-repository https://packages.microsoft.com/ubuntu/20.04/prod
```

Modify /etc/apt/sources.list. At the bottom of the file, change this:

```
deb https://packages.microsoft.com/ubuntu/20.04/prod focal main
# deb-src https://packages.microsoft.com/ubuntu/20.04/prod focal main
```
to:
```
deb [arch=amd64] https://packages.microsoft.com/ubuntu/20.04/prod focal main
# deb-src [arch=amd64] https://packages.microsoft.com/ubuntu/20.04/prod focal main
```

```
sudo apt-get update
sudo apt install k4a-tools
sudo apt install libk4a1.4-dev
```
If that doesn't work, compile from source :

```
wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.3/libk4a1.3_1.3.0_amd64.deb
wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.3-dev/libk4a1.3-dev_1.3.0_amd64.deb
wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4abt1.0/libk4abt1.0_1.0.0_amd64.deb
wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4abt1.0-dev/libk4abt1.0-dev_1.0.0_amd64.deb
wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.3/libk4a1.3_1.3.0_amd64.deb
wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/k/k4a-tools/k4a-tools_1.3.0_amd64.deb

```
Install all above packages with sudo ```dpkg -i 'package'``` 

```
git clone https://github.com/microsoft/Azure_Kinect_ROS_Driver.git
sudo apt install ninja-build

mkdir build && cd build
cmake .. -GNinja
ninja
sudo ninja install
```

## ros openpose
*from https://github.com/ravijo/ros_openpose*

```
mkdir $HOME/pepper_teleop/catkin_ws/src
cd $HOME/pepper_teleop/catkin_ws/src
git clone https://github.com/ravijo/ros_openpose.git
cd $HOME/pepper_teleop/catkin_ws
catkin_make

```
run with
```
roslaunch ros_openpose run.launch camera:=azurekinect
```
