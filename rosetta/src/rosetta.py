import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Int64
from thruster.msg import gypseas
from thruster.msg import dolphins
import pickle


class rosetta :

  def __init__(self):

    rospy.init_node('rosetta', anonymous=False)
    
    print(rospy.get_name())
    
    self.t1=0
    self.t2=0
    self.t3=0
    self.t4=0
    self.d1=0
    self.d2=0
    self.d3=0
    self.d4=0

    self.PBLDC_1 = rospy.Publisher('/prop_1_to_thruster_1_joint_controller/command', Float64, queue_size=1000)
    self.PBLDC_2 = rospy.Publisher('/prop_2_to_thruster_2_joint_controller/command', Float64, queue_size=1000)
    self.PBLDC_3 = rospy.Publisher('/prop_3_to_thruster_3_joint_controller/command', Float64, queue_size=1000)
    self.PBLDC_4 = rospy.Publisher('/prop_4_to_thruster_4_joint_controller/command', Float64, queue_size=1000)
    self.PBLDC_5 = rospy.Publisher('/prop_5_to_thruster_5_joint_controller/command', Float64, queue_size=1000)
    self.PBLDC_6 = rospy.Publisher('/prop_6_to_thruster_6_joint_controller/command', Float64, queue_size=1000)
    self.PBLDC_7 = rospy.Publisher('/prop_7_to_thruster_7_joint_controller/command', Float64, queue_size=1000)
    self.PBLDC_8 = rospy.Publisher('/prop_8_to_thruster_8_joint_controller/command', Float64, queue_size=1000)
    
    self.rate = rospy.Rate(10)

    self.gypseas=rospy.Subscriber("converter", gypseas, self.talker1)
    self.dolphins=rospy.Subscriber("converter", dolphins, self.talker2)
    
    rospy.spin()


  def converter(self,x):
    # X=pwm Y=rpm
    coeffs = pickle.load(open('src/translation_layer/pickle/coeffs.pickle', 'rb'))
    if (x>1470 and x<1530):
      0
    else:
      y= x**3*coeffs[0] + x**2*coeffs[1] + x*coeffs[2] + coeffs[3]
    return y
  

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

      self.PBLDC_1.publish(self.converter(self.t1))
      self.PBLDC_2.publish(self.converter(self.t2))
      self.PBLDC_3.publish(self.converter(self.t3))
      self.PBLDC_4.publish(self.converter(self.t4))
      self.PBLDC_5.publish(self.converter(self.d1))
      self.PBLDC_6.publish(self.converter(self.d2))
      self.PBLDC_7.publish(self.converter(self.d3))
      self.PBLDC_8.publish(self.converter(self.d4))

      self.rate.sleep()

if __name__=='__main__':

  try:
      x = rosetta()
      x.start()
  except rospy.ROSInterruptException:
      pass


  
