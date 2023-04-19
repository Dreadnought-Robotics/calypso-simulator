#! /usr/bin/python3
import rospy
from calypso_msgs.msg import gypseas
from sensor_msgs.msg import Imu
import numpy as np
from pid_calypso import pid as PID
from geometry_msgs.msg import Quaternion
from scipy.interpolate import interp1d
from tf.transformations import euler_from_quaternion
import time
from scipy.interpolate import interp1d 
# from scipy.integrate import trapezoid

class pid_gypseas:
  
  def __init__(self):

    rospy.init_node('gypseas_pid', anonymous=False)


    self.roll=PID()
    self.pitch=PID()
    self.heave=PID()
    self.yaw=PID()

    self.pitch.k=[1,0.5,0.05]
    self.roll.k=[1,0.5,0.05]
    self.heave.k=[1,0.5,0.005]
          
    self.throttle1 = 1580
    self.throttle2 = 1580
    self.throttle3 = 1580
    self.throttle4 = 1580


    self.m = interp1d([0, 75],[1574,2000])
    self.n = interp1d([-75, 0], [1500,1574])

    self.rate = rospy.Rate(10)
    self.gypseas_publisher = rospy.Publisher('/rosetta/gypseas', gypseas, queue_size=100)

    self.start_time = time.time()
  
  def gypseas_pid(self):

    CORD=rospy.Subscriber("/calypso_sim/heading", Quaternion, self.Heading_subscriber)

    PID_pitch = PID.getPID(self.pitch,True)
    PID_roll = PID.getPID(self.roll,True)
    PID_heave = PID.getPID(self.heave, False)

    # if we wanna map linear motiion: 
    # PID_heave_ = PID.getPID(self.heave, False)
    # if PID_heave_>=0:
    #   PID_heave = self.m(PID_heave_)
    # else:
    #   PID_heave = self.n(PID_heave_)
    # for testing gyro PID, comment the PID_heave part.

    a=[0,90]
    b=[1580,1800]
    self.g=gypseas()
    self.g.t1 = round(self.roll.interpolate((PID_roll - PID_pitch + PID_heave),a,b))
    self.g.t2 = round(self.roll.interpolate((- PID_roll - PID_pitch + PID_heave),a,b))
    self.g.t3 = round(self.roll.interpolate((- PID_roll + PID_pitch + PID_heave),a,b))
    self.g.t4 = round(self.roll.interpolate((PID_roll + PID_pitch + PID_heave),a,b))

    # self.g=gypseas()
    # self.g.t1 = round(PID_heave + PID_roll - PID_pitch)
    # self.g.t2 = round(PID_heave - PID_roll - PID_pitch)
    # self.g.t3 = round(PID_heave - PID_roll + PID_pitch)
    # self.g.t4 = round(PID_heave + PID_roll + PID_pitch)

    #print("PID-roll : ", PID_roll ," PID-pitch : ",PID_pitch, " PID-heave : ", PID_heave)
    print("current z :",self.heave.current_position)
    print("goal z :",self.heave.final)
    print(self.g)
    self.gypseas_publisher.publish(self.g)
    self.rate.sleep()
      
  def Imu_subscriber(self, Imu):

    time_elapsed =  Imu.header.stamp.secs-self.start_time
    
    self.pitch.time.append(time_elapsed)
    self.roll.time.append(time_elapsed)
    self.heave.time.append(time_elapsed)
    self.heave.acc.append(Imu.linear_acceleration.z)

    self.roll.current_vel= Imu.angular_velocity.x
    self.pitch.current_vel = Imu.angular_velocity.y
    self.yaw.current_vel= Imu.angular_velocity.z
    self.roll.current_position,self.pitch.current_position,self.yaw.current_position= PID.convert(Imu.orientation.x,Imu.orientation.y,Imu.orientation.z,Imu.orientation.w)

    self.heave.current_position=self.heave.get_current_pose()

    self.gypseas_pid()
    
  
  def Heading_subscriber(self,pose):
    self.heave.final=pose.z

  def getgyp(self, gypseas):
    self.throttle1 = gypseas.t1
    self.throttle2 = gypseas.t2
    self.throttle3 = gypseas.t3
    self.throttle4 = gypseas.t4
  
  def start(self):
    
    try:
      IMU = rospy.Subscriber("/calypso_sim/imu/data", Imu, self.Imu_subscriber)
      rospy.spin()
    except:
      print("ros shut down")

if __name__=='__main__':
  try:
      x = pid_gypseas()
      x.start()
  except rospy.ROSInterruptException:
      pass