# RoboPuppet Project
WPI Major Qualifying Project 2019-2020

## Contributors
- Dan Oates
- Mike Silder
- Tabby Gibbs

## Install Instructions
These instructions are exclusively for ROS Kinetic on Ubuntu 16.04.

### Update Git Submodules
- Run terminal in the repo root
- Update git submodules:
```
git submodule update --init
```

### Add Teensy UDEV Rules
- Copy text from: https://www.pjrc.com/teensy/49-teensy.rules
- Place file in: /etc/udev/rules.d/49-teensy.rules:
```
cd /etc/udev/rules.d
sudo touch 49-teensy.rules
sudo gedit 49-teensy.rules
```
- Reboot your PC for the change to take effect

### Install Baxter Simulator
- Install package dependencies:
```
sudo apt-get install gazebo7 ros-kinetic-qt-build ros-kinetic-gazebo-ros-control ros-kinetic-gazebo-ros-pkgs ros-kinetic-ros-control ros-kinetic-control-toolbox ros-kinetic-realtime-tools ros-kinetic-ros-controllers ros-kinetic-xacro python-wstool ros-kinetic-tf-conversions ros-kinetic-kdl-parser
```
- Run terminal in repo root
- Install Baxter simulator packages:
```
cd Catkin/src
wstool init .
wstool merge https://raw.githubusercontent.com/RethinkRobotics/baxter_simulator/kinetic-devel/baxter_simulator.rosinstall
wstool update
```
- Build source:
```
cd ..
source /opt/ros/kinetic/setup.bash
catkin_make
```
- Open Baxter startup script:
```
gedit src/baxter/baxter.sh
```
- Change 'your_ip' on line 26 to "127.0.0.1"
- Change 'ros_version' on line 30 to "kinetic"

### Test Simulator with Teensy
- Install Visual Studio Code and the PlatformIO extension
- Make sure the Teensy is still plugged in
- Open the Firmware project in Platformio and open platformio.ini
- Set the following build flags (comment and uncomment as needed):
```
; Build Flags
build_flags =
	-D PLATFORM_ARDUINO				; Arduino platform [Platform.h]
	-D PLATFORM_3V3					; 3.3V board [Platform.h]
	-D I2CDEVICE_BUFFER_SIZE=8		; I2C buffer size [I2CDevice.h]
	-D SERIALSERVER_TX_MAX_IDS=20	; Max TX messages [SerialServer.h]
	-D SERIALSERVER_RX_MAX_IDS=20	; Max RX messages [SerialServer.h]
	-D SERIALSERVER_TX_MAX_LEN=20	; Max TX message length [SerialServer.h]
	-D SERIALSERVER_RX_MAX_LEN=20	; Max RX message length [SerialServer.h]
```
- Build and upload the project to the Teensy
- Run in terminal in Catkin directory:
```
source devel/setup.bash
./src/baxter/baxter.sh sim
roslaunch robopuppet main_sim.launch
```
- Verify Baxter is running:
```
rostopic hz /robot/state
```
- The state topic should be publishing at 100Hz
- Move the RoboPuppet arms and verify that simulated Baxter follows
