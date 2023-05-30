# UAV Concepts Exploration

## UAV vs Drone

A drone is an unmanned aircraft or ship that is guided remotely or autonomously whereas a UAV is simply an Unmanned Aerial Vehicle, something that can fly without a pilot onboard.

![alt text](https://identifiedtech.wpenginepowered.com/wp-content/uploads/2016/04/blog_drone-vs-uav3.jpg)

## IMU Sensor

An IMU (inertial measurement unit) is a device that combines inertial sensors – gyroscopes and accelerometers – to provide acceleration and orientation data that can be used to calculate position and velocity. Some models of IMU also incorporate magnetometers, which outputs measurements of the Earth’s magnetic field that can be used to improve the accuracy of orientation measurements. IMUs typically have one of each sensor per axis being measured, up to a maximum of three axes for measuring roll, pitch and yaw.

IMUs are used for a variety of applications in UAVs and drones. They allow the aircraft to maintain stability and control while experiencing high winds or performing steep turning manoeuvres. They can also be used to enable highly accurate station-keeping or autonomous waypoint following.

It is used to measure force, magnetic field and angular rate. These sensors include
* 3-axis accelerometer
* 3-axis gyroscope
* 3-axis magnetometer (optional)

Addition of the magentometer it a 9-axis IMU, or else it can be considered as a 6-axis IMU.

### Accelerometer

![alt text](https://www.digikey.com/en/articles/~/media/Images/Article%20Library/TechZone%20Articles/2011/May/Using%20An%20Accelerometer%20for%20Inclination%20Sensing/TZS111_Using_An_Fig_10.jpg)


## Inertial Navigation System (INS)
Also known as Inertial Guidance System, used in UAVs, USVs and UGVs.

Inertial Navigation is a self-referential method by which a system may track its own position, orientation and velocity (once the initial values for these parameters are provided) without the need of external refernces liek landmarks, celestial objects, satellites etc.

For instance, if the acceleration of a body is known, mathematical integration will give the velocity. In this manner, accelerometers can be used for inertial navigation.

The orientation of the system can be tracked using gyroscopes, which measure angular rate (change in angular velocity). 

Magnetometers can be used to measure the strength and direction of the earth's magnetic field, and can be used to calculate the system yaw and heading.

![alt text](https://www.researchgate.net/publication/348803228/figure/fig7/AS:984434408955905@1611718861641/a-Pitch-yaw-and-roll-angles-of-an-aircraft-with-body-orientation-O-u-v-original.ppm)



## References 

### Basics
* https://www.identifiedtech.com/blog/uav-surveying/drone-technology-ending-the-drone-vs-uav-debate-drone-basics-101/#:~:text=A%20drone%20is%20an%20unmanned,named%20for%20it's%204%20propellers.

### IMU sensor
* https://www.unmannedsystemstechnology.com/expo/inertial-measurement-units-imu/#:~:text=An%20IMU%20(inertial%20measurement%20unit,to%20calculate%20position%20and%20velocity.
* 
#### Accelerometer
* https://www.digikey.com/en/articles/using-an-accelerometer-for-inclination-sensing
* 

### INS
* https://www.smlease.com/entries/mechanical-design-basics/what-is-the-difference-between-roll-pitch-yaw-aircraft-motions/
* https://www.researchgate.net/figure/a-Pitch-yaw-and-roll-angles-of-an-aircraft-with-body-orientation-O-u-v-original_fig7_348803228
* 

### Pixhawk
* https://ardupilot.org/copter/docs/common-px4fmu-overview.html
* https://docs.px4.io/main/en/flight_controller/pixhawk_series.html
* https://robotics.stackexchange.com/questions/5053/why-does-the-pixhawk-have-2-imus#:~:text=This%20is%20because%20the%20chances,IMUs%20all%20of%20different%20brands
* https://www.rcgroups.com/forums/showthread.php?1987175-Pixhawk-from-3D-Robotics-and-PX4/page483
* https://github.com/3drobotics/Pixhawk_OS_Hardware/blob/master/FMUv3_REV_G/Schematic%20Print/Schematic%20Prints.PDF

