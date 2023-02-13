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


## ros openpose
*from https://github.com/ravijo/ros_openpose*

```
```

```
```
