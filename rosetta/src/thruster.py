import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Int64
from calypso_msgs.msg import gypseas
from calypso_msgs.msg import dolphins
import math
import pickle


class rosetta :

  def __init__(self):

    rospy.init_node('rosetta_thruster', anonymous=False)
    
    print(rospy.get_name())
    
    self.t1=0
    self.t2=0
    self.t3=0
    self.t4=0
    self.d1=0
    self.d2=0
    self.d3=0
    self.d4=0

    self.PBLDC_1 = rospy.Publisher('/thruster_1_controller/command', Float64, queue_size=1000)
    self.PBLDC_2 = rospy.Publisher('/thruster_2_controller/command', Float64, queue_size=1000)
    self.PBLDC_3 = rospy.Publisher('/thruster_3_controller/command', Float64, queue_size=1000)
    self.PBLDC_4 = rospy.Publisher('/thruster_4_controller/command', Float64, queue_size=1000)
    self.PBLDC_5 = rospy.Publisher('/thruster_5_controller/command', Float64, queue_size=1000)
    self.PBLDC_6 = rospy.Publisher('/thruster_6_controller/command', Float64, queue_size=1000)
    self.PBLDC_7 = rospy.Publisher('/thruster_7_controller/command', Float64, queue_size=1000)
    self.PBLDC_8 = rospy.Publisher('/thruster_8_controller/command', Float64, queue_size=1000)
    
    self.rate = rospy.Rate(10)

  def converter(self,x):
  
    if (x>=1472 and x<=1528):
      y=0
    
    else:
      y = 873445.5884049138 + -3060.828578480122*x + 4.270869048814241*x**2 + -0.002944908465733391*x**3 + 1.0007672335895403e-06*x**4 + -1.3418449357438108e-10*x**5
    
    if x==0:
      y=0

    if x<1472:
       y=-y

    return y *0.1047198

  # def old_converter(self,x):
  #   # X=pwm Y=rpm
  #   coeffs = list()
  #   coeffs.append(-1.6453836430727448e-05)
  #   coeffs.append(0.07440821248059681)
  #   coeffs.append(-100.45437634228745)
  #   coeffs.append(38769.58439545923)
  #   # [-1.6453836430727448e-05, 0.07440821248059681, -100.45437634228745, 38769.58439545923]
  #   # print(coeffs)
  #   if (x>1470 and x<1530):
  #     y = 0

  #   else:
  #     y = x**3*coeffs[0] + x**2*coeffs[1] + x*coeffs[2] + coeffs[3] 

  #   if (x<1000 or x>2000):
  #     y = 0
  #   # print(x,y)
    
  #   return round(round(y, 4)*0.1047198,4)
  

  def talker1(self,msg_gypseas):

    self.t1=msg_gypseas.t1
    self.t2=msg_gypseas.t2
    self.t3=msg_gypseas.t3
    self.t4=msg_gypseas.t4  

  def talker2(self,msg_dolphins):

    self.d1=msg_dolphins.d1
    self.d2=msg_dolphins.d2
    self.d3=msg_dolphins.d3
    self.d4=msg_dolphins.d4
    
  def start(self):

    while True:
      self.gypseas=rospy.Subscriber("/rosetta/gypseas", gypseas, self.talker1)
      self.dolphins=rospy.Subscriber("/rosetta/dolphins", dolphins, self.talker2)
      self.t=Float64()
      self.t.data=self.converter(self.t1)
      self.PBLDC_1.publish(self.t)
      self.t.data=self.converter(self.t2)
      self.PBLDC_2.publish(self.t)
      self.t.data=self.converter(self.t3)
      self.PBLDC_3.publish(self.t)
      self.t.data=self.converter(self.t4)
      self.PBLDC_4.publish(self.t)
      self.t.data=self.converter(self.d1)
      self.PBLDC_5.publish(self.t)
      self.t.data=self.converter(self.d2)
      self.PBLDC_6.publish(self.t)
      self.t.data=self.converter(self.d3)
      self.PBLDC_7.publish(self.t)
      self.t.data=self.converter(self.d4)
      self.PBLDC_8.publish(self.t)
      
      self.rate.sleep()

if __name__=='__main__':

  try:
      x = rosetta()
      x.start()
  except rospy.ROSInterruptException:
      pass


  
