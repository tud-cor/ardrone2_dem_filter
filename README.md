# ardrone2_dem_filter
ROS package for simulation and physical flight control of the Parrot AR.Drone 2.0 quadrotor and implementation of DEM state estimation.

This repository serves two goals:
1. Be able to quickly run all packages required to fly the Parrot AR.Drone 2.0 quadrotor in Gazebo simulation or in a lab environment.
2. Be able to easily perform data analysis of the recorded flight data in MATLAB. This includes data pre-processing, running the conventional Kalman filter and DEM state estimator algorithms and interpreting the results.

The filter results are discussed in [this](http://resolver.tudelft.nl/uuid:156157c6-d7f0-4dc1-a55a-b2e4ed66f1c2) master thesis document.


Installation instructions
-------------------------
All software in this repository is tested with Ubuntu 16.04, ROS Kinetic and Gazebo 7.16.
It is assumed that [ROS Kinetic](https://wiki.ros.org/kinetic/Installation/Ubuntu) and [Gazebo](http://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros#A.InstallPre-BuiltDebians) are already installed.

To use this repository, execute the following commands:
```
$ mkdir -p ~/ardrone2_ws/src
$ cd ~/ardrone2_ws/src
$ vcs import --input https://raw.githubusercontent.com/tud-cor/ardrone2_dem_filter/master/ardrone2_dem_filter.repos
$ cd ..
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic
$ catkin_make
$ source ~/ardrone2_ws/devel/setup.bash
$ echo "source ~/ardrone2_ws/devel/setup.bash" >> ~/.bashrc
```


You are ready to simulate or fly the physical drone and analyze its data! Please take a look below for the steps to take.


Run simulator
-------------
To simulate the Parrot AR.Drone 2.0, execute the following commands in different terminal tabs (open a new tab using `ctrl+shift+t`):
```
$ roscore
$ roslaunch ardrone2_dem_filter ardrone2.launch
```
This will spawn the AR.Drone 2.0 quadrotor in an empty Gazebo environment and open the tum\_ardrone GUI.

Fill in the following text in the GUI text field and click the Send button:
```
setInitialReachDist 0.2
setStayWithinDist 0.2
setStayTime 0.1

takeoff

goto 0 0 1 0
goto -1 -1 1 0
goto -1 1 1 0
goto 1 1 1 0
goto 1 -1 1 0
goto 0 0 1 0

land
```
This will set the reference frame, ensure drone takeoff, perform controlled drone flight in a square with sides of 2 meter and land the drone.

### Keyboard and joystick control
It is also possible to control the drone using a keyboard and/or using a joystick (see *README.md* in the tum\_ardrone package). These control methods will be selected when pressing an arbitrary key or button, respectively.
The following keyboard commands apply:
- **s** Takeoff
- **d** Land
- **F1** Emergency (simulation: "Reset quadrotor!!" is printed; physical flight: turn engines off and fall, press again to try to recover to normal state -> immediately take over using joystick)
- **j/l** Roll
- **i/k** Pitch
- **u/o** Yaw
- **q/a** Gaz

In order to add manual control using a joystick, first determine the joystick device number in Ubuntu. Therefore, open a new terminal tab and run `$ ls /dev/input`. Then connect the joystick to the computer and run again `$ ls /dev/input`. If it is properly connected, a new *jsX* device should appear, which is the joystick.
The joystick can be tested by running the following command (where *X* represents the number as determined):
```
$ jstest /dev/input/jsX
```
Using the axes and buttons on the joystick should give changing values on the screen (16 bit signed integers for axes and on/off for buttons). Check whether these values are logical.
Now open a new terminal and run the following commands:
```
$ rosparam set joy_node/dev "/dev/input/jsX"
$ rosrun joy joy_node
```
This stores your joystick number in the ROS parameter database and launches the joystick node, which publishes a message with the current joystick axes and buttons states once an axes or button value changes. This can be seen by opening a new terminal tab, typing `$ rostopic echo joy` and moving joytisck axes and pressing joystick buttons. If this functions correctly, you can use the joystick to control the drone in simulation.
The following joystick commands apply:
- **LB** Takeoff (release this button to land)
- **RB** Emergency
- **Left joystick** Left/right: roll, up/down: pitch
- **Right joystick** Left/right: yaw, up/down: gaz

### Adding wind to simulation
It is also possible to apply wind in simulation by adding the `enable_wind` parameter to the roslaunch command in the following way:
```
$ roslaunch ardrone2_dem_filter ardrone2.launch enable_wind:=true
```
The wind settings (i.e. starting time, duration, force, etc.) can be set in *quadrotor\_with\_wind.urdf.xacro* in the directory *tum\_simulator/cvg\_sim\_gazebo/urdf*.


If everything works fine in simulation, congratulations! You can proceed to the physical flight!


Run physical flight
-------------------
Only fly the drone in a controlled lab environment with a properly functioning OptiTrack camera system (for drone state feedback) and keyboard/joystick (for manual control in case of emergency, see **Keyboard and joystick control** above)!

To fly the Parrot AR.Drone 2.0:
1. Turn on the drone.
2. Connect with PC to the drone wifi network.
3. Execute the following commands in separate terminal tabs:
```
$ roscore
$ roslaunch ardrone2_dem_filter ardrone2.launch sim:=false
```
Now the ardrone\_autonomy package will be used to communicate with the drone over wifi. You can control the drone the same way as in simulation.

If one changes the network configuration and assigns a different IP address to the drone, this address can be added to the *roslaunch* command. For example, to use the IP address 192.168.2.4, execute the following command:
```
$ roslaunch ardrone2_dem_filter ardrone2.launch sim:=false droneip:=192.168.2.4
```
ROS bags can be recorded using the *rosbag* package at the end of the launch file. You can change this code according to your own wishes.


If physical flight works as desired, good luck with your experiments!


Data analysis
-------------
All data analysis software is tested using MATLAB R2019a with the following packages installed (this list is sufficient, not checked on necessity): Curve Fitting Toolbox 3.5.9, Phased Array System Toolbox 4.1, Global Optimization Toolbox 4.1, Wavelet Toolbox 5.2, Econometrics Toolbox 5.2, System Identification Toolbox 9.10, Robotics System Toolbox Interface for ROS Custom Messages 19.1.0, Symbolic Math Toolbox 8.3, Statistics and Machine Learning Toolbox 11.5, Simulink Control Design 5.3, Simulink 9.3, Signal Processing Toolbox 8.2, Robotics System Toolbox 2.2, Optimization Toolbox 8.3, Instrument Control Toolbox 4.0, Image Processing Toolbox 10.4, DSP System Toolbox 9.8, Deep Learning Toolbox 12.1 and Control System Toolbox 10.6.

For data analysis in MATLAB, first add the matlab directory to the MATLAB path using the MATLAB Command Window:
```
addpath('~/ardrone2_ws/src/ardrone2_dem_filter/matlab')
```
The MATLAB code consists of three parts:
1. Data processing (*matlab* directory).
2. Filter execution (*matlab/demCode* directory).
3. Unit tests for data interpolation functionality (*matlab/unitTests* directory).

### Data processing
The data processing part contains scripts, functions and data storage files that are used to:
- Convert recorded experiment data in the form of bag files to interpolated data in the form of .mat files (using *getExpData.m* and *getFilterInputs.m*).
- Interpret the filter data resulting from running the code in the *matlab/demCode* directory as shown below (using *plotFilterSSE.m* and *plotImpactGenCoords.m*).
- Analyze the noises present in the experimental system (using *getMeasNoise.m*, *getGyroRollNoise.m*, *getFilterInputs.m*, *analyzeColNoise.m*, *gaussianFilterValidity.m* and *deriveSmoothnessFromPrecision.m*).
- Derive the Parrot AR.Drone 2.0 thrust coefficients (using *eqThrustCoefCalcAndPlot.m*, *eqPwmCalcAndCheck.m* and *eqPwmDisp.m*).
- Create flight trajectories (using *createFlightplans.m*).
- Show spiky data coming from tum\_simulator (using *plotTumSimulatorSpikyData.m*).

The bag files required to run some of the MATLAB scripts are included in this repository in the directory *bagfiles*. Each (unzipped) bag file should be stored in the *~/.ros* directory on your PC in order for the MATLAB scripts to run without errors.

### Filter execution
The filter execution part is mainly based on the code in the [DEM_observer repository](https://github.com/ajitham123/DEM_observer). All settings can be found in *DEM_observer.m*, except for the dataset (defined in *get_ardrone2_flight_data.m*). To execute the desired filters, run *DEM_observer.m*.

### Unit tests for data interpolation functionality
These unit tests are used to test custom interpolation functionality that is used to process recorded experimental data. To execute the unit tests, run *runTests.m*.


Acknowledgements
----------------
This repository is largely based on the following repositories:
- [ROS Kinetic version](https://github.com/rdelgadov/tum_ardrone/tree/kinetic-devel) of the [tum\_ardrone](https://github.com/tum-vision/tum_ardrone) package.
- [ROS Kinetic version](https://github.com/angelsantamaria/tum_simulator/tree/kinetic) of the [tum\_simulator](https://github.com/tum-vision/tum_simulator) package.
- Wind plugin of the [rotors\_simulator](https://github.com/ethz-asl/rotors_simulator) repository.
- [DCSC version](https://github.com/hai-zhu/mocap_optitrack/tree/dcsc_pose_stamped) of the [mocap\_optitrack](https://github.com/hai-zhu/mocap_optitrack) repository.
- [DEM_observer](https://github.com/ajitham123/DEM_observer) repository.
