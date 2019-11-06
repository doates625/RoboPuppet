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
gedit baxter/baxter.sh
```
- Change 'your_ip' on line 26 to "127.0.0.1"
- Change 'ros_version' on line 30 to "kinetic"
- Open RoboPuppet launch file:
```
gedit robopuppet/launch/RoboPuppet.xml
```
- Change value="..." on line 13 to PORT

### Test Simulator with Teensy
- Make sure the Teensy is still plugged in
- Open the MainProc project in Platformio and open platformio.ini
- Set the following build flags (comment and uncomment as needed):
```
; Build Flags
build_flags =
	-D PLATFORM_ARDUINO		; Platform library flag
	-D PLATFORM_TEENSY		; Platform library flag
	-D PLATFORM_3V3			; Platform operating voltage
	-D STUB_I2C				; Stubs I2C communication
	;-D STUB_SERIAL			; Stubs ROS serial communication
	-D BAXTER_STUB_DEMO		; Demonstration of Teensy control
	-D LTIFILTER_MAX_A=10	; Max A coefficients for discrete filter
	-D LTIFILTER_MAX_B=10	; Max B coefficients for discrete filter
```
- Build and upload the project to the Teensy
- Run in terminal in Catkin directory:
```
source devel/setup.bash
./src/baxter/baxter.sh sim
roslaunch robopuppet RoboPuppet.xml
```
- Verify Baxter is running:
```
rostopic echo /robot/state
```
- The state topic should be publishing at 100Hz
- If everything was done correctly, Baxter should be periodically raising and lowering its arms every few seconds
