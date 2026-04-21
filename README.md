<p align="center">

  <h1 align="center"> Breaking the Static Assumption: A Dynamic-Aware LIO Framework (测试)
  </h1>

[comment]: <> (  <h2 align="center">PAPER</h2>)
  <h3 align="center">
  <a href="https://github.com/arclab-hku/btsa">Original Github</a> 
  </h3>


 ```bash
# 安装livox
cd ~/catkin_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver.git
cd ..
catkin build livox_ros_driver

# 安装btsa
cd ~/catkin_ws/src
git clone git@github.com:R-C-Group/btsa_test.git
cd ..
# catkin clean btsa
catkin build btsa
source devel/setup.bash

# 其他依赖安装
sudo apt-get install libgoogle-glog-dev
# 但系统自动安装的可能有问题，修复了“CMakeLists.txt”
``` 


* 运行：
```bash
source devel/setup.bash
roslaunch btsa dynamic.launch
```