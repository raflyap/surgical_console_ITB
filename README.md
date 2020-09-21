# surgical_console_ITB

Repository for Surgical Console Capstone Design in Electrical Engineering Department of Institut Teknologi Bandung.
The development repo can be found at: code lama di: https://github.com/raflyap/master_console


Each folder represents the package in ROS Framework. So, you need to install ROS to implement the programs.

Requirement:
- Installation of ROS
- Installation of COPPELIA sim
- Installation of Rosserial (Modify the serial_node.py as instructed below to run 2 rosserial nodes).


# HOW TO SETUP SERIAL NODE

Langkah mengubah serial_nopde biar bisa 2 arduino

# HOW TO SETUP SIMULATOR
# Installing and Running CoppeliaSim from scratch (tested Ubuntu 16.04 ROS kinetic)
## Installing
1. Download CoppeliaSim and Extract at /home : https://www.coppeliarobotics.com/ubuntuVersions
2.Download and Extract sim_ros_interface and LibPlugins from :
```
[simExtROSInterface](https://github.com/CoppeliaRobotics/simExtROSInterface)
and
[libPlugin](https://github.com/CoppeliaRobotics/libPlugin)
```
3. Make ROS workspace with catkin_make
4. Copy sim_ros_interface to /$workspace_ws/src/
5. Copy libPlugin to /CoppeliaSimDIR/programming/libPlugins/
7. Make sure you have minimum version of CMake is 16.0. If you don't have it, run this code:
```
version=3.17
build=3
mkdir ~/temp
cd ~/temp
wget https://cmake.org/files/v$version/cmake-$version.$build.tar.gz
tar -xzvf cmake-$version.$build.tar.gz
cd cmake-$version.$build/
./bootstrap
make -j4
sudo make install
```
and verify with this:
```cmake --version```

8. At terminal, execute this code at ROS workspace that has been built before
```
export COPPELIASIM_ROOT_DIR=~/path/to/coppeliaSim/folder
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release -DLIBPLUGIN_DIR=$COPPELIASIM_ROOT_DIR/programming/libPlugin
```
***if there some error with make[2]: *** [generated/stubs.cpp] Error 1. Try to install xsltproc with : sudo apt install xsltproc***
9. At workspace_ws directory, search for libsimExtROSInterface.so and copy to CoppeliaSim/

## Running
1. Advance to CoppeliaSim directory.
2. Run this code, make sure ROSInterface is in log console. 
```./coppeliaSim.sh```
3. Add *TA192001004_simulator.ttt* file to COPPELIADIR/scene/
3. Advance to File->Open Scene-> TA192001004_simulator.ttt
4. Enjoy

# Ethernet things
## Setting up ubuntu (if using usb to ethernet connector)
1. Connect two computer with ethernet cable
2. Check both ip addresses's of two computer. In ubuntu you can use ```ifconfig``` and search for usb to ethernet. It's common named by *enx00e04c534458* 
3. If ubuntu doesn't identify its ipv4 addresses, run ```sudo gedit /etc/network/interfaces``` and add this line
```
auto enx00e04c534458
iface enx00e04c534458 inet static
    address 169.254.60.165
    netmask 255.255.0.0
```
4. Finally, restart the network-manager : ```sudo service network-manager restart```

## Setting up ip address
1. Use ip address that use coppeliasim (as a client to the code client.py at socket_simulator package 

## Run simulator
At server (computer that serve data)
1. ```rosrun socket_programming server.py```
2. (optional) ```rosrun socket_programming generator.py``` for generate dummy values

At client (simulator)
1. ```rosrun socket_simulator client.py```
