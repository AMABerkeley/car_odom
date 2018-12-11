# car_odom

This package requires an Inertial Navigation System ROS topic as an Odometry Message type. It also uses a classical bike model to do velocity estimation of the 4-WD car.

It outputs a topic at /odom that fuses INS + Bike model. 
