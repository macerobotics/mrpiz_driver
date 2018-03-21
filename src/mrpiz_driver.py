#!/usr/bin/env python

# MRPiZ ROS driver
# Version : 0.2
# Date : 21/03/2018

import rospy
import paramiko
import numpy as np
from cv_bridge.core import CvBridge
from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point, Quaternion
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import math
import tf
import time
from mrpizSshDriver import *

laser_frequency = 40
num_readings = 100


class MRPiZDriver(object):

  def __init__(self, ip_robot):

    self._name = "mrpiz_robot_0"
    self.Orientation_pos = 0
    self.x_pos = 0
    self.y_pos = 0
    self.deltaSteps = 1
    self.deltaOrientation = 1
    self.ipRobot= ip_robot
    

    self.distance_sensors_msg = []
    self.distance_sensors_pub = []


    self.start_Time = time.time()
    self.end_Time = time.time()

    self.transformBroad = tf.TransformBroadcaster()

  '''---------------------'''
  ''' connection to MRPiZ robot with SSH '''
  def connection_to_mrpiz(self):
    connexionToMRPiZ(self.ipRobot)
    ssh_resetUc()

  '''-------Reception command velocite--------------'''
  def get_cmd_vel(self, data):
    x = data.linear.x
    y = data.linear.y
    angle = data.angular.z
    self.send_cmd_to_mrpiz(x,angle) 

  '''-------Envoie command au robot MRPiZ --------------'''
  def send_cmd_to_mrpiz(self, x, angular):
    right = int((x + angular) * 1000)
    left = int((x - angular) * 1000)
    print "right = ", right
    print "left = ", left

    # Commande forward
    if((right > 1)and(left >  1)):
     ssh_forward(right)

    elif((right > 1)and(left <  0)):
      ssh_turnLeft(right)

    elif((right < 0)and(left >  1)):
      ssh_turnRight(-right)

    elif((right < 1)and(left <  1)):
       ssh_back(-right)

    elif((right < 1)and(right > -1)and(left < 1)and(left > -1)):
      ssh_stop()



  '''------- Mise a jour des capteurs --------------'''
  def update_sensors_mrpiz(self):

    print "update MRPiZ robot sensors"

    
    ################## distance sensors ############################################"
    sensor1 = ssh_proximity(1)# read distance sensor 1
    sensor2 = ssh_proximity(2)
    sensor3 = ssh_proximity(3)
    sensor4 = ssh_proximity(4)
    sensor5 = ssh_proximity(5)
    
    # transform millimeter value to meter 
    sensor1 = sensor1/1000
    sensor2 = sensor2/1000
    sensor3 = sensor3/1000
    sensor4 = sensor4/1000
    sensor5 = sensor5/1000

    
    # sensor1
    self.distance_sensors_msg[0].range = sensor1       # valeur lu
    self.distance_sensors_msg[0].header.stamp = rospy.Time.now()
    self.distance_sensors_pub[0].publish(self.distance_sensors_msg[0])

    # MRPiZ distance sensor positions (cm)
    # S1(x, y, z)
    # S1(3.5,3.0, 3.0)
    # 0.71 = angle en radian
    # sendTransform => position du capteur en metre
    self.transformBroad.sendTransform((-0.0358, 0.030, 0.030), tf.transformations.quaternion_from_euler(0, 0, 2.43), rospy.Time.now(), "/base_prox0", "/base_link")

    # sensor2
    self.distance_sensors_msg[1].range = sensor2       # valeur lu
    self.distance_sensors_msg[1].header.stamp = rospy.Time.now()
    self.distance_sensors_pub[1].publish(self.distance_sensors_msg[1])

    # 
    self.transformBroad.sendTransform((-0.0198, 0.043, 0.030), tf.transformations.quaternion_from_euler(0, 0, 2.01), rospy.Time.now(), "/base_prox1", "/base_link")

    # sensor3
    self.distance_sensors_msg[2].range = sensor3       # valeur lu
    self.distance_sensors_msg[2].header.stamp = rospy.Time.now()
    self.distance_sensors_pub[2].publish(self.distance_sensors_msg[2])

    self.transformBroad.sendTransform((0.000, 0.037, 0.030), tf.transformations.quaternion_from_euler(0, 0, 1.57), rospy.Time.now(), "/base_prox2", "/base_link")
    
    # sensor4
    self.distance_sensors_msg[3].range = sensor4       # valeur lu
    self.distance_sensors_msg[3].header.stamp = rospy.Time.now()
    self.distance_sensors_pub[3].publish(self.distance_sensors_msg[3])

    self.transformBroad.sendTransform((0.0198, 0.043, 0.030), tf.transformations.quaternion_from_euler(0, 0, 1.13), rospy.Time.now(), "/base_prox3", "/base_link")

    # sensor5
    self.distance_sensors_msg[4].range = sensor5       # valeur lu
    self.distance_sensors_msg[4].header.stamp = rospy.Time.now()
    self.distance_sensors_pub[4].publish(self.distance_sensors_msg[4])

    self.transformBroad.sendTransform((0.0358, 0.030, 0.030), tf.transformations.quaternion_from_euler(0, 0, 0.71), rospy.Time.now(), "/base_prox4", "/base_link")



    
    ################## odometry ######################################
    
    self.x_pos = ssh_robotPositionX() # read robot position x
    self.y_pos = ssh_robotPositionY() # read robot position y
    self.Orientation_pos = ssh_robotPositionO() # read robot orientation

    # conversion en metre
    self.x_pos = self.x_pos/1000
    self.y_pos = self.y_pos/1000

    odometry_msg = Odometry()
    odometry_msg.header.stamp = rospy.Time.now()
    odometry_msg.header.frame_id = "odom"
    odometry_msg.child_frame_id = "/base_link"

    # ATTENTION, le x et y sont inverser !
    odometry_msg.pose.pose.position = Point(self.y_pos, self.x_pos, 0) # TEST POSITION!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    quater = tf.transformations.quaternion_from_euler(0, 0, self.Orientation_pos)
    odometry_msg.pose.pose.orientation = Quaternion(*quater)
    self.end_Time = time.time()
    odometry_msg.twist.twist.linear.x = self.deltaSteps / (self.end_Time-self.start_Time) 
    odometry_msg.twist.twist.angular.z = self.deltaOrientation / (self.end_Time-self.start_Time)    
                                                                                          
    self.start_Time = self.end_Time
    self.odometry_pub.publish(odometry_msg)

     ###################################### Transform position robot ######################################

    position_robot = (odometry_msg.pose.pose.position.x, odometry_msg.pose.pose.position.y, odometry_msg.pose.pose.position.z)
    orientation_robot = (odometry_msg.pose.pose.orientation.x, odometry_msg.pose.pose.orientation.y, odometry_msg.pose.pose.orientation.z, odometry_msg.pose.pose.orientation.w)

    self.transformBroad.sendTransform(position_robot, orientation_robot, odometry_msg.header.stamp, odometry_msg.child_frame_id, odometry_msg.header.frame_id)
   

    # la position des roues est defini dans le fichier urdf
    self.transformBroad.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "/right_wheel", "/base_link")
    self.transformBroad.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "/left_wheel", "/base_link")
    self.transformBroad.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "/pi_board", "/base_link")
    
    ###################################### Laser scan ######################################
    scan = LaserScan()
    scan.header.stamp = rospy.Time.now()
    scan.header.frame_id = 'laser_frame'
    scan.angle_min = -1.57
    scan.angle_max = 1.57
    scan.angle_increment = 3.14/20
    #scan.time_increment = (1.0 / laser_frequency) / (num_readings)
    scan.range_min = 0.0
    scan.range_max = 0.02 # 20 cm


  '''-------- close connection to MRPiZ robot --------'''
  def disconnect(self):
    connexionToMRPiZClose()


  '''------------RUN---------'''
  def run(self):

    # # Disconnect when rospy is close
    rospy.on_shutdown(self.disconnect)


    rospy.Subscriber('cmd_vel', Twist, self.get_cmd_vel)

    # distance sensor publisher 
    for c in range(0,5):
      self.distance_sensors_pub.append(rospy.Publisher("distance_sensors"+str(c), Range))
      self.distance_sensors_msg.append(Range())
      self.distance_sensors_msg[c].radiation_type = Range.INFRARED
      self.distance_sensors_msg[c].header.frame_id = "/base_prox" + str(c)
      self.distance_sensors_msg[c].field_of_view = 0.25 
      self.distance_sensors_msg[c].min_range = 0.001	# 0.1 cm
      self.distance_sensors_msg[c].max_range = 0.255	# 20 cm

    # odometry publisher 
    self.odometry_pub = rospy.Publisher('odom', Odometry)

    # laser scan publisher 
    self.laserScan_pub = rospy.Publisher('scan', LaserScan)

    self.connection_to_mrpiz()

    print "wait..."
    time.sleep(2)
    print "connexion to MRPiZ ok."

    
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
       self.update_sensors_mrpiz()# mise a jour des capteurs
       rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()



'''--------------------------------------------------------------------------------------------------------'''
'''--------------------------------------------------------------------------------------------------------'''
'''--------------------------------------------------------------------------------------------------------'''
'''--------------------------------------------------------------------------------------------------------'''
def run():
  rospy.init_node("mrpiz_drive", anonymous=True)
  MRPiZDriver("192.168.42.2").run()

if __name__ == "__main__":
  run()
