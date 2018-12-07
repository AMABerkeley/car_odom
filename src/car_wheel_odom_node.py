#!/usr/bin/env python
# license removed for brevity
import rospy
import tf
import math
from math import sin, cos, pi
#TODO : Import Odometry from nav_msgs
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_msgs.msg import UInt16
from std_msgs.msg import Float32
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

global current_speed
global gps
global current_steering_angle

current_steering_angle = 0
current_speed = 0
gps=False

def velocity_callback(msgs):
    global current_speed
    current_speed = msgs.data

def steering_callback(msgs):
    global current_steering_angle
    current_steering_angle = msgs.data
    
def listener():

    rospy.init_node('odometry_publisher', anonymous=True)
    # rospy.Subscriber("velocity", UInt16, velocity_control)


    rospy.Subscriber("speed", Float32, velocity_callback)
    rospy.Subscriber("steering_angle", Float32, steering_callback)

    # rospy.Subscriber("velocity_gps", UInt16,)
    #rospy.Subscriber("throttle", UInt16)
    
def car_control():
    global current_speed
    global gps
    global current_steering_angle

    # pub_th = rospy.Publisher('servo_th', UInt16, queue_size=10)
    # pub_st = rospy.Publisher('servo_st', UInt16, queue_size=10)
    # pub_speed = rospy.Publisher('speed', Float32, queue_size=10)
    # pub_st_ang = rospy.Publisher('steering_angle', Float32, queue_size=10)

    odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()
    wheelbase_ = .290
    current_angular_velocity = current_speed * tan(current_steering_angle) / wheelbase_
    x_ = 0.0
    y_ = 0.0
    yaw_ = 0.0


    current_time = rospy.get_time()
    last_time = rospy.get_time()
    rate = rospy.Rate(10) # 10hz


    while not rospy.is_shutdown():

        current_time = rospy.get_time()
        #compute odometry in a typical way given robot velocities
        
        dt = current_time - last_time

        x_dot = current_speed * cos(yaw_)
        y_dot = current_speed * sin(yaw_)

        x_ += x_dot * dt
        y_ += y_dot * dt

        yaw_ += current_angular_velocity * dt

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, yaw_)

        odom_broadcaster.sendTransform(
            (x_,y_,0),
            odom_quat,
            current_time,
            "base_link",
            "odom"
            )

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.pose.pose = Pose(Point(x_, y_, 0), Quaternion(*odom_quat))
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(x_dot, y_dot, 0), Vector3(0, 0, current_angular_velocity))
        odom_pub.publish(odom)

        last_time = current_time
        r.sleep()


        # velocity_pwm = velocity * 30 + 1475
        # steering_pwm = steering_angle * 10 + 1450


        # if gps==True:
        #     velocity_pwm += gain*(velocity_gps - velocity)

        # rospy.loginfo("throttle pwm")
        # rospy.loginfo(int(round(velocity_pwm)))
        # pub_th.publish(int(round(velocity_pwm)))
        # rospy.loginfo("steering pwm")
        # rospy.loginfo(int(round(steering_pwm)))
        # pub_st.publish(int(round(steering_pwm)))

        # rospy.loginfo("speed")
        # rospy.loginfo(velocity)
        # pub_speed.publish(velocity)
        # rospy.loginfo("steering angle")
        # rospy.loginfo(steering_angle)
        # pub_st_ang.publish(steering_angle)
       

        # rate.sleep()



if __name__ == '__main__':
    try:
        listener()
        car_control()
    except rospy.ROSInterruptException:
        pass