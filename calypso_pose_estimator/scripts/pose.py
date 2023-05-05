import rospy
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
from scipy.integrate import trapezoid
import time
from pid_calypso import pid 
import numpy as np
from tf.transformations import euler_from_quaternion

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
      
      vel = round(self.integrate(self.acc,self.time), 2)
      self.vel.append(vel)
      return self.integrate(self.vel, self.time)


class pose_estimator:
  
  def __init__(self):

    rospy.init_node("pose_estimator")

    self.start_time=time.time()

    self.current_pose=rospy.Publisher("/calypso_sim/pose",Quaternion, queue_size=10)

    self.x_cord=cord()
    self.y_cord=cord()
    self.z_cord=cord()

    self.rate = rospy.Rate(10) 

  
  def Imu_subscriber(self, Imu):
    
    time_elapsed = time.time()-self.start_time
    self.x_cord.time.append(time_elapsed)
    self.y_cord.time.append(time_elapsed)
    self.z_cord.time.append(time_elapsed)
    self.x_cord.acc.append(Imu.linear_acceleration.x)
    self.y_cord.acc.append(Imu.linear_acceleration.y)
    self.z_cord.acc.append(round(Imu.linear_acceleration.z, 3))
    self.pose=Quaternion()
    self.pose.x=self.x_cord.get_current_pose()
    self.pose.y=self.y_cord.get_current_pose()
    self.pose.z=self.z_cord.get_current_pose()
    x,y,z= np.degrees(euler_from_quaternion([Imu.orientation.x,Imu.orientation.y,Imu.orientation.z,Imu.orientation.w]))
    print("({},{},{}) ({},{},{})".format(self.pose.x,self.pose.y,self.pose.z,x,y,z))

    self.current_pose.publish(self.pose)
  
  def start(self):
    imu=rospy.Subscriber("/calypso_sim/imu/data",Imu, self.Imu_subscriber)
    rospy.spin()

if __name__ == "__main__":

  try:
    p=pose_estimator()
    p.start()
  except Exception as e:
    print(e)