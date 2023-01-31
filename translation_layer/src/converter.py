import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Int64
from thruster.msg import gypseas
from thruster.msg import dolphins

def converter(x):
  # X=pwm Y=rpm
  y=5.923734234998961e-19*x**7 + -3.951181510642254e-18*x**6 + x**5*1.3024703556215067e-10 + x**4*-9.73095354449366e-07 + x**3*0.0028687891784984322 + x**2*-4.168692152450146 + x*2994.0940134416433 + -856530.1524953379
  return y

def talker1(msg_gypseas):

  PBLDC_1.publish(converter(msg_gypseas.t2))
  PBLDC_2.publish(converter(msg_gypseas.t1))
  PBLDC_3.publish(converter(msg_gypseas.t4))
  PBLDC_4.publish(converter(msg_gypseas.t3))

def talker2(msg_dolphins):

  PBLDC_5.publish(converter(msg_dolphins.d1))
  PBLDC_6.publish(converter(msg_dolphins.d3))
  PBLDC_7.publish(converter(msg_dolphins.d4))
  PBLDC_8.publish(converter(msg_dolphins.d2))
  
def listener1():
  rospy.Subscriber("converter", gypseas, talker1)
  rospy.Subscriber("converter", dolphins, talker2)
  rospy.spin()

if __name__=='__main__':
  
  rospy.init_node('converter', anonymous=True)

  PBLDC_1 = rospy.Publisher('/prop_1_to_thruster_1_joint_controller/command', Float64, queue_size=1000)
  PBLDC_2 = rospy.Publisher('/prop_2_to_thruster_2_joint_controller/command', Float64, queue_size=1000)
  PBLDC_3 = rospy.Publisher('/prop_3_to_thruster_3_joint_controller/command', Float64, queue_size=1000)
  PBLDC_4 = rospy.Publisher('/prop_4_to_thruster_4_joint_controller/command', Float64, queue_size=1000)
  PBLDC_5 = rospy.Publisher('/prop_5_to_thruster_5_joint_controller/command', Float64, queue_size=1000)
  PBLDC_6 = rospy.Publisher('/prop_6_to_thruster_6_joint_controller/command', Float64, queue_size=1000)
  PBLDC_7 = rospy.Publisher('/prop_7_to_thruster_7_joint_controller/command', Float64, queue_size=1000)
  PBLDC_8 = rospy.Publisher('/prop_8_to_thruster_8_joint_controller/command', Float64, queue_size=1000)
  listener1() 
