#! /usr/bin/python3
from calypso_msgs.msg import gypseas
from calypso_msgs.msg import dolphins
import rospy

if __name__ == '__main__':

  rospy.init_node("rosetta_test",anonymous=False)
  rate = rospy.Rate(10)
  #gpub=rospy.Publisher("/calypso_pid/topple_checker",gypseas,queue_size=1000)
  gpub=rospy.Publisher("/rosetta/gypseas",gypseas,queue_size=1000)

  dpub=rospy.Publisher("/rosetta/dolphins",dolphins,queue_size=1000)
  g=gypseas()
  g.t1=1556
  g.t2=1556
  g.t3=1556
  g.t4=1556
  d=dolphins()
  d.d1=1500
  d.d2=1500
  d.d3=1500
  d.d4=1500
  while(True):
    gpub.publish(g)
    dpub.publish(d)
    print("done")
    rate.sleep()




