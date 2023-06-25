# Set Up


## Virtual machine and ROS
### Option 1: VM with ROS Noetic, ROS 2 Foxy, Gazebo simulator, TurtleBot3 exemples, Matlab integration plugin, etc.
Already comes with many of the necessary packages and setups but also other unsecessary things.
- Follow the instructions on [this](https://fr.mathworks.com/support/product/robotics/ros2-vm-installation-instructions-v6.html) page.

### Option 2: Basic Ubuntu 20.04 and ROS Noetic
This option has more steps but should take less time and memory.
 
- Download a Ubuntu 20.04 desktop image.  
- Follow the instructions from the tutorial in Option 1 but instead of downloading the archive containing the VM use the image from previous step.
- Follow [this](https://wiki.ros.org/noetic/Installation/Ubuntu) tutorial to install ROS Noetic in the virtual machine.
- Install and build Gazeboplugin
```bash
cd GazeboPlugin/ 
mkdir build && cd build
cmake ..
make

echo 'export SVGA_VGPU10=0' >> ~/.bashrc 
echo 'export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/GazeboPlugin/export' >> ~/.bashrc
```


## Simulation
Virtual machine ros noetic terminal
````bash
sudo apt update
sudo apt upgrade
sudo apt-get install ros-noetic-ros-controllers ros-noetic-gazebo* ros-noetic-moveit* ros-noetic-industrial-core
sudo apt install ros-noetic-dynamixel-sdk ros-noetic-dynamixel-workbench*
sudo apt install ros-noetic-robotis-manipulator
````

```bash
mkdir <your_instalation_path>/src 
cd <your_instalation_path>/src
git clone https://gitlab-student.centralesupelec.fr/p19-arm/tossing-bot.git
git clone https://gitlab-student.centralesupelec.fr/p19-arm/open_manipulator.git
git clone https://gitlab-student.centralesupelec.fr/p19-arm/open_manipulator_dependencies.git
git clone https://gitlab-student.centralesupelec.fr/p19-arm/open_manipulator_msgs.git
git clone https://gitlab-student.centralesupelec.fr/p19-arm/open_manipulator_simulations.git

# Gripper fix
git clone https://github.com/JenniferBuehler/general-message-pkgs.git
git clone https://github.com/JenniferBuehler/gazebo-pkgs.git

cd ..
catkin_make


```

```bash
# You can add those to your .bashrc so you won't need to repeat it everytime  
source <your_instalation_path>/devel/setup.bash
```

## Computer vision

Should we put it on the same directory as the others? do we need to use make?
```bash
cd <your_instalation_path>/src
git clone https://gitlab-student.centralesupelec.fr/p19-arm/computer-vision.git
```


## Matlab control

On your windowns terminal:
```bash 
git clone https://gitlab-student.centralesupelec.fr/p19-arm/matlab.git
```


# Use

On your windows machine open the cloned matlab repository and run the programs <ADD LIST OF PROGRAMS>

You may enonter an error caused by bus variable length. If so, change *Dimensions* and *DimensionsMode* of Gazebo_SL_Bus_gazebo_msgs_JointState.joint_position and Gazebo_SL_Bus_gazebo_msgs_JointState.joint_velocity. Go to Simulink® Toolstrip: On the Modeling tab, in the Design gallery, click Type Editor. There you can set *DimenstionsMode* to Fixed and *Dimensions* to 1. 


On your virtual machine terminal do
```bash 
roslaunch tossing-bot complete.launch
```

And ina seconnd terminal
```bash 
rosrun tossing-bot main.py
```

After the first launch you will be asked if a new launch should be done and the new position for the cup. 