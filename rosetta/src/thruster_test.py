from calypso_msgs import gypseas
from calypso_msgs import dolphins
import rospy

if __name__ == '__main__':

  rospy.init_node("rosetta_test",False)
  gpub=rospy.Publisher("/rosetta/gypseas",1000)
  dpub=rospy.Publisher("/rosetta/dolphins",1000)
  g=gypseas()
  g.t1=1500
  g.t2=1500
  g.t3=1500
  g.t4=1500
  d=dolphins()
  d.d1=1500
  d.d2=1500
  d.d3=1500
  d.d4=1500
  while(True):
    gpub.publish(g)
    dpub.publish(d)
    rospy.spin()

    


