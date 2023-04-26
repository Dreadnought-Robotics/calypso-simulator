import rospy
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
from scipy.integrate import trapezoid
import time
from pid_calypso import pid 
import numpy as np

class cord:

  def __init__ (self):
    
      self.time=[]
      self.final=0
      self.acc = []
      self.vel = []
  
  def integrate(self, y, x):
      try:
          return trapezoid(y, x)
      except Exception as e:
          print(len(y),len(x))
          print("Sync failure !!")
  
  def get_current_pose(self):
      
      vel = self.integrate(self.acc,self.time)
      self.vel.append(vel)
      return  self.integrate(self.vel,self.time)


class pose_estimator:
  
  def __init__(self):

    rospy.init_node("pose_estimator")

    self.start_time=time.time()

    self.current_pose=rospy.Publisher("/calypso_sim/pose",Quaternion, queue_size=10)

    self.x_cord=cord()
    self.y_cord=cord()
    self.z_cord=cord()

    self.pose=Quaternion()
    self.rate = rospy.Rate(10) 

  
  def Imu_subscriber(self, Imu):
    
    time_elapsed =  time.time()-self.start_time
    self.x_cord.time.append(time_elapsed)
    self.y_cord.time.append(time_elapsed)
    self.z_cord.time.append(time_elapsed)
    self.x_cord.acc.append(Imu.linear_acceleration.x)
    self.y_cord.acc.append(Imu.linear_acceleration.y)
    self.z_cord.acc.append(Imu.linear_acceleration.z)

    self.pose.x=self.x_cord.get_current_pose()
    self.pose.y=self.y_cord.get_current_pose()
    self.pose.z=self.z_cord.get_current_pose()

    print("acc:",self.z_cord.acc[-1],"vel :", self.z_cord.vel[-1]," z :",self.pose.z)
    self.current_pose.publish(self.pose)

    #print("time :{t} x: {x_val} y: {y_val} z: {z_val}".format(t=self.time[-1],x_val=pose.x,y_val=pose.y,z_val=pose.z))


    # if len(self.acc_x)>self.mem:
      
    #   self.prev_x=pose.x
    #   self.prev_y=pose.y
    #   self.prev_z=pose.z
    #   self.acc_x=[]
    #   self.acc_y=[]
    #   self.acc_z=[]
    #   self.vel_x=[]
    #   self.vel_y=[]
    #   self.vel_z=[]
    #   self.time=[]

  
  def start(self):
    imu=rospy.Subscriber("/calypso_sim/imu/data",Imu, self.Imu_subscriber)
    rospy.spin()

if __name__ == "__main__":

  try:
    p=pose_estimator()
    p.start()
  except Exception as e:
    print(e)