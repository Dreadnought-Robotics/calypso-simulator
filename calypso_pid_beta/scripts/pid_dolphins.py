#! /usr/bin/python3
import rospy
from calypso_msgs.msg import dolphins
from sensor_msgs.msg import Imu
from pid_calypso import pid as PID
from geometry_msgs.msg import Quaternion
import time

class pid_dolphins:

  def __init__(self):

    rospy.init_node('dolphins_pid', anonymous=False)

    self.time_lapsed = 0

    self.yaw=PID()
    self.surge=PID()
    self.sway=PID()

    self.yaw.k=[2,0.5,0.05]
    self.surge.k=[2,0.5,0.005]
    self.sway.k=[2,0.5,0.005]

    self.throttle1 = 1650
    self.throttle2 = 1650
    self.throttle3 = 1650
    self.throttle4 = 1650

    self.rate = rospy.Rate(10)
    self.dolphins_publisher = rospy.Publisher('/rosetta/dolphins', dolphins, queue_size=100)

    self.start_time = time.time()

  def dolphins_pid(self):

    CORD=rospy.Subscriber("/calypso_sim/heading", Quaternion, self.Heading_subscriber)

    
    print("roll")
    print(self.yaw)

    PID_yaw = PID.getPID(self.yaw,True)
    PID_surge = PID.getPID(self.surge,False)
    PID_sway = PID.getPID(self.sway,False)

    #Incase we are using one direction thrust
    # #Using One direction thrust for surge
    # if PID_surge_temp>0:
    #    PID_surge_pos = PID_surge_temp
    #    PID_surge_neg = 0
    # else:
    #    PID_surge_pos = 0
    #    PID_surge_neg = PID_surge_temp

    # #Using One direction thrust for sway
    # if PID_sway_temp>0:
    #    PID_sway_pos = PID_sway_temp
    #    PID_sway_neg = 0
    # else:
    #    PID_sway_pos = 0
    #    PID_sway_neg = PID_sway_temp

    # self.d=dolphins()
    # self.d.d1 = round(self.throttle1 + PID_yaw + PID_surge_neg + PID_sway_neg)
    # self.d.d2 = round(self.throttle2 - PID_yaw + PID_surge_neg + PID_sway_neg)
    # self.d.d3 = round(self.throttle3 + PID_yaw + PID_surge_pos + PID_sway_pos)
    # self.d.d4 = round(self.throttle4 - PID_yaw + PID_surge_pos + PID_sway_pos)

    self.d=dolphins()
    self.d.d1 = round(self.throttle1 + PID_yaw + PID_surge - PID_sway)
    self.d.d2 = round(self.throttle2 - PID_yaw + PID_surge + PID_sway)
    self.d.d3 = round(self.throttle3 + PID_yaw - PID_surge - PID_sway)
    self.d.d4 = round(self.throttle4 - PID_yaw - PID_surge + PID_sway)

    print("PID-yaw : ",PID_yaw)
    print("PID-surge : ",PID_surge, "PID-sway : ", PID_sway)

    self.dolphins_publisher.publish(self.d)
    self.rate.sleep()

  def Imu_subscriber(self, Imu):

    time_elapsed = Imu.header.stamp.secs-self.start_time

    self.surge.time.append(time_elapsed)
    self.sway.time.append(time_elapsed)
    self.yaw.time.append(time_elapsed)
    self.surge.acc.append(Imu.linear_acceleration.x)
    self.sway.acc.append(Imu.linear_acceleration.y)
  
    self.yaw.current_vel= Imu.angular_velocity.z
    x,y,self.yaw.current_position= PID.convert(Imu.orientation.x,Imu.orientation.y,Imu.orientation.z,Imu.orientation.w)
    
    self.surge.current_position=self.surge.get_current_pose()
    self.sway.current_position=self.sway.get_current_pose()

    self.dolphins_pid()

  def start(self):

    try:
      IMU = rospy.Subscriber("/calypso_sim/imu/data", Imu, self.Imu_subscriber)
      rospy.spin()
    
    except :
      print("ros shut down !!!")
  
  def Heading_subscriber(self,pose):
    
    self.yaw.final=pose.w
    self.surge.final=pose.x
    self.sway.final=pose.y

if __name__=='__main__':

  try:
      x = pid_dolphins()
      x.start()
  except rospy.ROSInterruptException:
      pass