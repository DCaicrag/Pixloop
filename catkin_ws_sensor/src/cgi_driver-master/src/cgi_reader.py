#!/usr/bin/python2.7

import rospy
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from serial import Serial
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler
import pyproj
import math


port = rospy.get_param("serial_port", '/dev/ttyUSB1')
baud = rospy.get_param("serial_baud", 230400)
utm_id = rospy.get_param("utm_id", 48)

gravity = 9.80665
proj_84 = pyproj.Proj(init='epsg:4326')
proj_utm = pyproj.Proj(init='epsg:32650')

interface = Serial(port=port, baudrate=baud)

def gpchc_parser(sentence):
    try:
        split_sentence = sentence.split(',')
    except:
        return
    if split_sentence[0] == "$GPCHC":
        # parse Euler angles
        yaw = float(split_sentence[3])-90
        pitch = float(split_sentence[4])
        roll = float(split_sentence[5])
        # parse angular velocity
        angular_velocity_x = float(split_sentence[7])
        angular_velocity_y= -float(split_sentence[6])
        angular_velocity_z = float(split_sentence[8])
        # parse linear acc
        linear_acceleration_x = float(split_sentence[10])
        linear_acceleration_y = -float(split_sentence[9])
        linear_acceleration_z = float(split_sentence[11])
        # parse lat, lon, alt
        latitude = float(split_sentence[12])
        longitude = float(split_sentence[13])
        altitude = float(split_sentence[14])
        # parse velocity
        linear_velocity_x = float(split_sentence[15])
        linear_velocity_y = float(split_sentence[16])
        linear_velocity_z = float(split_sentence[17])
        vehicle_velocity = float(split_sentence[18])

        # gnss status
        status = int(split_sentence[21])
        return yaw, pitch, roll, angular_velocity_x, angular_velocity_y, angular_velocity_z, linear_acceleration_x, linear_acceleration_y, linear_acceleration_z, latitude, longitude, altitude, linear_velocity_x, linear_velocity_y, linear_velocity_z, vehicle_velocity, status
    else:
        pass

def get_sentence():
    return interface.readline()

def cgi_publisher():
    rospy.init_node('cgi_driver_node', anonymous=True)
    pub_fix = rospy.Publisher('/gps/fix', NavSatFix, queue_size=1)
    pub_imu = rospy.Publisher('/ch_imu/data', Imu, queue_size=1)
    pub_twist = rospy.Publisher('/gps/twist', TwistStamped, queue_size=1)
    pub_odom = rospy.Publisher('/gps/odom', Odometry, queue_size=1)

    fix_msg =  NavSatFix()
    fix_msg.header.frame_id = 'gps'
    imu_msg = Imu()
    imu_msg.header.frame_id = 'imu'
    twist_msg = TwistStamped()
    twist_msg.header.frame_id = 'gps'
    odom_msg = Odometry()
    odom_msg.header.frame_id = 'gps'

    while not rospy.is_shutdown():
        time_stamp = rospy.Time.now()
        try:
            # parse data
            parse_data = gpchc_parser(get_sentence())

            linear_vel_x = parse_data[12]
            linear_vel_y = parse_data[13]
            linear_vel_z = parse_data[14]

            q = quaternion_from_euler(math.radians(parse_data[2]), math.radians(parse_data[1]), math.radians(parse_data[0]))

            linear_acc_x = parse_data[6]*gravity
            linear_acc_y = parse_data[7]*gravity
            linear_acc_z = parse_data[8]*gravity

            angular_vel_x = parse_data[3]
            angular_vel_y = parse_data[4]
            angular_vel_z = parse_data[5]

            lat = parse_data[9]
            lon = parse_data[10]
            alt = parse_data[11]
            x, y = pyproj.transform(proj_84, proj_utm, lon, lat)

            # navsat msg
            fix_msg.header.stamp = time_stamp
            fix_msg.latitude = lat
            fix_msg.longitude = lon
            fix_msg.altitude = alt

            # imu msg
            imu_msg.header.stamp = time_stamp
            imu_msg.linear_acceleration.x = linear_acc_x 
            imu_msg.linear_acceleration.y = linear_acc_y
            imu_msg.linear_acceleration.z = linear_acc_z
            imu_msg.orientation.x = q[0]
            imu_msg.orientation.y = q[1]
            imu_msg.orientation.z = q[2]
            imu_msg.orientation.w = q[3]
            imu_msg.angular_velocity.x = angular_vel_x
            imu_msg.angular_velocity.y = angular_vel_y
            imu_msg.angular_velocity.z = angular_vel_z

            # twist msg
            twist_msg.header.stamp = time_stamp
            twist_msg.twist.linear.x = linear_vel_x
            twist_msg.twist.linear.y = linear_vel_y
            twist_msg.twist.linear.z = linear_vel_z
            twist_msg.twist.angular.x = angular_vel_x
            twist_msg.twist.angular.y = angular_vel_y
            twist_msg.twist.angular.z = angular_vel_z

            # odometry msg
            odom_msg.header.stamp = time_stamp
            odom_msg.pose.pose.position.x = x
            odom_msg.pose.pose.position.y = y
            odom_msg.pose.pose.position.z = alt
            odom_msg.pose.pose.orientation.x = imu_msg.orientation.x
            odom_msg.pose.pose.orientation.y = imu_msg.orientation.y
            odom_msg.pose.pose.orientation.z = imu_msg.orientation.z
            odom_msg.pose.pose.orientation.w = imu_msg.orientation.w

            odom_msg.twist.twist.linear = twist_msg.twist.linear
            odom_msg.twist.twist.angular = twist_msg.twist.angular

            # publish data
            pub_imu.publish(imu_msg)
            pub_fix.publish(fix_msg)
            pub_twist.publish(twist_msg)
            pub_odom.publish(odom_msg)

        except:
            pass
    
if __name__ == '__main__':
    cgi_publisher()
