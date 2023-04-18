#! /usr/bin/python3
import rospy
from calypso_msgs.msg import gypseas
from sensor_msgs.msg import Imu
import numpy as np
from pid_calypso import pid as PID
from geometry_msgs.msg import Quaternion
from scipy.interpolate import interp1d
from tf.transformations import euler_from_quaternion
# from scipy.integrate import trapezoid

class pid_gypseas:
  
  def __init__(self):

    rospy.init_node('gypseas_pid', anonymous=False)


    self.roll=PID()
    self.pitch=PID()
    self.heave=PID()
    self.yaw=PID()

    self.pitch.k=[2,0.5,0.05]
    self.roll.k=[2,0.5,0.05]
    self.heave.k=[2,0.5,0.005]
          
    self.throttle1 = 1650
    self.throttle2 = 1650
    self.throttle3 = 1650
    self.throttle4 = 1650


    self.m = interp1d([0, 75],[1574,2000])
    self.n = interp1d([-75, 0], [1500,1574])

    self.rate = rospy.Rate(10)
    self.gypseas_publisher = rospy.Publisher('/rosetta/gypseas', gypseas, queue_size=100)

    self.start_time = 0
    self.time_imu = 0
    self.acc_heave = 0
  
  def start(self):

    while not rospy.is_shutdown():
      
      IMU = rospy.Subscriber("/calypso_sim/imu/data", Imu, self.Imu_subscriber)
      CORD=rospy.Subscriber("/calypso_sim/heading", Quaternion, self.Heading_subscriber)
      


      # for simulator, since we dont know how much we have to move, we integrate imu values to know current posiotion, which will later be changed using camera input
      # CORD = 3
      # self.heave.final = CORD
      # self.heave.acc.append(self.acc_heave)
      # self.vel = self.heave.integrate(self.heave.acc, self.heave.time)
      # self.heave.vel.append(self.vel)
      # self.heave.current_position = self.heave.integrate(self.heave.vel, self.heave.time)
      # print("current position : ", self.heave.current_position)
      
      time_elapsed = self.time_imu-self.start_time
      self.pitch.time_elapsed=time_elapsed
      self.roll.time_elapsed=time_elapsed
      self.heave.time_elapsed=time_elapsed
    
      print("Roll : ",self.roll," Pitch : ",self.pitch)

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

      self.g=gypseas()
      self.g.t1 = round(self.throttle1 + PID_roll - PID_pitch + PID_heave)
      self.g.t2 = round(self.throttle2 - PID_roll - PID_pitch + PID_heave)
      self.g.t3 = round(self.throttle3 - PID_roll + PID_pitch + PID_heave)
      self.g.t4 = round(self.throttle4 + PID_roll + PID_pitch + PID_heave)

      # self.g=gypseas()
      # self.g.t1 = round(PID_heave + PID_roll - PID_pitch)
      # self.g.t2 = round(PID_heave - PID_roll - PID_pitch)
      # self.g.t3 = round(PID_heave - PID_roll + PID_pitch)
      # self.g.t4 = round(PID_heave + PID_roll + PID_pitch)

      print("PID-roll : ",PID_roll," PID-pitch : ",PID_pitch, " PID-heave : ", PID_heave)

      self.gypseas_publisher.publish(self.g)
      self.rate.sleep()
      
  def Imu_subscriber(self, Imu):
    self.roll.current_vel= Imu.angular_velocity.x
    self.pitch.current_vel = Imu.angular_velocity.y
    self.yaw.current_vel= Imu.angular_velocity.z
    self.roll.current_position,self.pitch.current_position,self.yaw.current_position= self.convert(Imu.orientation.x,Imu.orientation.y,Imu.orientation.z,Imu.orientation.w)
    self.time_imu = Imu.header.stamp.secs

    # if we are integrating for linear motion: 
    # self.acc_heave = Imu.linear_acceleration.z
    
  
  def Heading_subscriber(self,pose):
    self.heave.final=pose.z

  def convert(self,x,y,z,w):

    orientation_list = [x,y,z,w]
    (roll, pitch, yaw) = np.degrees(euler_from_quaternion (orientation_list))
    return roll, pitch, yaw

  def getgyp(self, gypseas):
    self.throttle1 = gypseas.t1
    self.throttle2 = gypseas.t2
    self.throttle3 = gypseas.t3
    self.throttle4 = gypseas.t4

if __name__=='__main__':
  try:
      x = pid_gypseas()
      x.start()
  except rospy.ROSInterruptException:
      pass