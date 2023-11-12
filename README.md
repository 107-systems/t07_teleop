<a href="https://107-systems.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/107-systems.png" width="15%"></a>
:floppy_disk: `t07_teleop`
==========================
[![Build Status](https://github.com/107-systems/t07_teleop/actions/workflows/ros2.yml/badge.svg)](https://github.com/107-systems/t07_teleop/actions/workflows/ros2.yml)
[![Spell Check status](https://github.com/107-systems/t07_teleop/actions/workflows/spell-check.yml/badge.svg)](https://github.com/107-systems/t07_teleop/actions/workflows/spell-check.yml)

Keyboard based teleoperation for the [T07](https://github.com/107-systems/T07) mobile outdoor robot.

#### How-to-build
* Install `gsl-lite`
```bash
git clone https://github.com/gsl-lite/gsl-lite && cd gsl-lite
mkdir build && cd build
cmake .. && make -j8
sudo make install
```
* Install `Catch2`
```bash
git clone https://github.com/catchorg/Catch2 && cd Catch2
mkdir build && cd build
cmake .. && make -j8
sudo make install
```
* Install `fmt`
```bash
git clone https://github.com/fmtlib/fmt && cd fmt
mkdir build && cd build
cmake -DFMT_TEST=OFF ..
make -j8
sudo make install
```
* Install `mp-units`
```bash
git clone https://github.com/mpusz/mp-units && cd mp-units
mkdir build && cd build
cmake -DMP_UNITS_AS_SYSTEM_HEADERS=ON -DMP_UNITS_BUILD_LA=OFF ..
make -j8
sudo make install
```
* Build with `colcon`
```bash
cd $COLCON_WS/src
git clone https://github.com/107-systems/t07_teleop
cd $COLCON_WS
source /opt/ros/humble/setup.bash
colcon build --packages-select t07_teleop
```

#### How-to-run
```bash
cd $COLCON_WS
. install/setup.bash
ros2 launch t07_teleop teleop.py
```

#### Interface Documentation
##### Published Topics
|     Default name      |        Type         | Description                  |
|:---------------------:|:-------------------:|------------------------------|
| `/motor/left/target`  | `std_msgs/Float32`  | Motor left set-point in m/s  | 
| `/motor/right/target` | `std_msgs/Float32`  | Motor right set-point in m/s |

##### Subscribed Topics
|     Default name      |        Type         | Description                  |
|:---------------------:|:-------------------:|------------------------------|

##### Parameters
|                      Name                      |       Default        | Description                                                             |
|:----------------------------------------------:|:--------------------:|-------------------------------------------------------------------------|
|               `motor_left_topic`               | `motor/left/target`  |                                                                         |
|              `motor_right_topic`               | `motor/right/target` |                                                                         |
