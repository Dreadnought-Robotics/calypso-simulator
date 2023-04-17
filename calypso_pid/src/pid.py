#! /usr/bin/python3
import rospy
from calypso_msgs.msg import gypseas
from sensor_msgs.msg import Imu
import numpy as np
from tf.transformations import euler_from_quaternion
from scipy.integrate import trapezoid
import time

class pid:

  def __init__(self):

    rospy.init_node('calypso_pid', anonymous=False)

    self.time_lapsed = 0

    self.pitch=[2,0.5,0.05]
    self.roll=[2,0.5,0.05]

    self.pid_i_pitch = 0
    self.pid_i_roll = 0
    self.previous_error_pitch = 0
    self.previous_error_roll = 0
          
    self.throttle1 = 1650
    self.throttle2 = 1650
    self.throttle3 = 1650
    self.throttle4 = 1650

    self.rate = rospy.Rate(10)
    self.pwmspeed = rospy.Publisher('/rosetta/gypseas', gypseas, queue_size=100)
    
    self.w=0
    self.x=0
    self.y=0
    self.z=0

    self.angvel_x = 0
    self.angvel_y = 0 
    self.angvel_z = 0
    self.error_list = []
    self.time = []
    self.start_time = self.time_imu

  
  def start(self):
  

    while not rospy.is_shutdown():
        # self.angvel = rospy.Subscriber("/imu/data", Imu, self.getvel)
        self.angvel = rospy.Subscriber("/imu/data", Imu, self.getvel)
        end_time = self.time_imu
        self.time_lapsed = end_time-self.start_time
        # self.gypseas=rospy.Subscriber("/calypso_pid/topple_checker",gypseas, self.getgyp)
        self.roll_error , self.pitch_error , self.yaw_error = self.convert()
        print("roll")
        print(self.roll)

        print("pitch")
        print(self.pitch)
        self.PID_pitch = self.getPID(self.pitch, self.pitch_error, 0, self.pid_i_pitch, self.angvel_x, 0)
        self.PID_roll = self.getPID(self.roll, self.roll_error, 0, self.pid_i_roll, self.angvel_y, 0)

        self.g=gypseas()
        self.g.t1 = round(self.throttle1 + self.PID_roll - self.PID_pitch)
        self.g.t2 = round(self.throttle2 - self.PID_roll - self.PID_pitch)
        self.g.t3 = round(self.throttle3 - self.PID_roll + self.PID_pitch)
        self.g.t4 = round(self.throttle4 + self.PID_roll + self.PID_pitch)

        # self.g.t1 = 1600
        # self.g.t2 = 1600
        # self.g.t3 = 1600
        # self.g.t4 = 1600
      
      
        print("PID-roll")
        print(self.PID_roll)
      
        print("PID-pitch")
        print(self.PID_pitch)

        self.pwmspeed.publish(self.g)

        self.rate.sleep()
      

  def getvel(self, Imu):
    self.angvel_x = Imu.angular_velocity.x
    self.angvel_y = Imu.angular_velocity.y
    self.angvel_z = Imu.angular_velocity.z
    self.x = Imu.orientation.x
    self.y = Imu.orientation.y
    self.z = Imu.orientation.z
    self.w = Imu.orientation.w
    self.time_imu = Imu.header.stamp.secs

    # print("self.x")
    # print(self.x)
    # print("self.y")
    # print(self.y)
    # print("self.z")
    # print(self.z)
    # print("self.w")
    # print(self.w)

  def getPID(self,k, actual, desired, pid_i, angvel, prev_vel):
      
    kp=k[0];ki=k[1];kd=k[2]

    error = desired - actual
    self.error_list.append(error)
    self.time.append(self.time_lapsed)
    pid_i = self.integrate(self.error_list, self.time)
    feedforward = angvel - prev_vel
    pid_p = kp*error
    # pid_d = kd*(error - previous_error)
    pid_d = kd*angvel

    if pid_i>max(90-pid_p-pid_d, 0):
        pid_i = max(90-pid_p-pid_d,0)
    elif pid_i<min(-90-pid_i-pid_d, 0):
        pid_i = min(-90-pid_p-pid_d,0)

    pid_i_final = ki*pid_i
    PID = pid_p + pid_i_final + pid_d + feedforward/self.time_lapsed
    prev_vel = angvel
    if(PID > 90):
        PID=90
    if(PID < -90):
        PID=-90
    return PID
  

  def integrate(self, y, x):
    return trapezoid(y, x)


  def convert(self):

    # ysqr = self.y * self.y

    # t0 = +2.0 * (self.w * self.x + self.y * self.z)
    # t1 = +1.0 - 2.0 * (self.x * self.x + ysqr)
    # X = np.degrees(np.arctan2(t0, t1))

    # t2 = +2.0 * (sel
    # f.w * self.y - self.z * self.x)
    # t2 = np.where(t2>+1.0,+1.0,t2)
    # #t2 = +1.0 if t2 > +1.0 else t2

    # t2 = np.where(t2<-1.0, -1.0, t2)
    # #t2 = -1.0 if t2 < -1.0 else t2
    # Y = np.degrees(np.arcsin(t2))

    # t3 = +2.0 * (self.w * self.z + self.x * self.y)
    # t4 = +1.0 - 2.0 * (ysqr + self.z * self.z)
    # Z = np.degrees(np.arctan2(t3, t4))

    # return X, Y, Z 

    orientation_list = [self.x, self.y, self.z, self.w]

    (roll, pitch, yaw) = np.degrees(euler_from_quaternion (orientation_list))

    return roll, pitch, yaw
  

  def getgyp(self, gypseas):
    self.throttle1 = gypseas.t1
    self.throttle2 = gypseas.t2
    self.throttle3 = gypseas.t3
    self.throttle4 = gypseas.t4

if __name__=='__main__':

  try:
      x = pid()
      x.start()
  except rospy.ROSInterruptException:
      pass