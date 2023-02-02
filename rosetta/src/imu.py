import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Int64
from calypso_imu import buoy
from gazebo_msgs import Imu
import pickle
import math


class rosetta :

  def __init__(self):

    rospy.init_node('rosetta_imu', anonymous=False)
    
    print(rospy.get_name())
    
    self.x=0
    self.y=0
    self.z=0
    self.roll=0
    self.pitch=0
    self.yaw=0

    self.PBLDC_1 = rospy.Publisher('/rosetta/imu', Float64, queue_size=1000)
    
    self.rate = rospy.Rate(10)
  
  def convert(self,w, x, y, z):
    
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = math.degrees(math.atan2(t3, t4))

    return X, Y, Z
    

  def talker1(self,imu):

    self.x=imu.orientation.x
    self.y=imu.orientation.y
    self.z=imu.orientation.z
    self.roll,self.pitch,self.yaw=self.convert(imu.orientation.w,self.x,self.y,self.z)
    
  def start(self):

    while True:

      self.b=buoy()
      self.b.x=self.x
      self.b.y=self.y
      self.b.z=self.z
      self.b.roll=self.roll
      self.b.pitch=self.pitch
      self.b.yaw=self.yaw

      self.imu=rospy.Subscriber("/calypso/imu", buoy, self.talker1)
      self.imu.publish(self.b)

      self.rate.sleep()
      rospy.spin()

if __name__=='__main__':

  try:
      x = rosetta()
      x.start()
  except rospy.ROSInterruptException:
      pass


  
