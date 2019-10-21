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

### Install ROS Arduino Library
- Run in terminal:
```
cd CppDeps
rosrun rosserial_arduino make_libraries.py .
```
- Open CppDeps/ros_lib/ArduinoHardware.h
- Delete lines 44-63:
```
#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(__MKL26Z64__)
  #if defined(USE_TEENSY_HW_SERIAL)
    #define SERIAL_CLASS HardwareSerial // Teensy HW Serial
  #else
    #include <usb_serial.h>  // Teensy 3.0 and 3.1
    #define SERIAL_CLASS usb_serial_class
  #endif
#elif defined(_SAM3XA_)
  #include <UARTClass.h>  // Arduino Due
  #define SERIAL_CLASS UARTClass
#elif defined(USE_USBCON)
  // Arduino Leonardo USB Serial Port
  #define SERIAL_CLASS Serial_
#elif (defined(__STM32F1__) and !(defined(USE_STM32_HW_SERIAL))) or defined(SPARK) 
  // Stm32duino Maple mini USB Serial Port
  #define SERIAL_CLASS USBSerial
#else 
  #include <HardwareSerial.h>  // Arduino AVR
  #define SERIAL_CLASS HardwareSerial
#endif
```
- Replace deleted code with:
```
#include <usb_serial.h>
#define SERIAL_CLASS usb_serial_class
```
- Open the MainProc project in PlatformIO
- Plug in and upload the project to a Teensy 4.0

### Add Teensy UDEV Rules
- Copy text from: https://www.pjrc.com/teensy/49-teensy.rules
- Place file in: /etc/udev/rules.d/49-teensy.rules:
```
cd /etc/udev/rules.d
sudo touch 49-teensy.rules
sudo gedit 49-teensy.rules
```
- Reboot your PC for the change to take effect

### Test Teensy ROS Communication
- Make sure the Teensy is still plugged in
- View the USB ports with:
```
ls /dev/tty*
```
- The PORT is either /dev/ttyUSBX or /dev/ttyACMX (where X is a number)
- Run ROS and echo the calibrated topic:
```
roscore
rosrun roserial_arduino serial_node.py PORT
rostopic hz /calibrated
```
- The topic should be published at 10Hz

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
- Run in terminal:
```
source devel/setup.bash
cp src/baxter/baxter.sh .
./src/baxter/baxter.sh sim
roslaunch robopuppet RoboPuppet.xml
```
- Verify Baxter is running:
```
rostopic echo /robot/state
```
- The state topic should be publishing at 100Hz
