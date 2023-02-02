import rospy
from thruster.msg import gypseas
from thruster.msg import dolphins

def talker():
    pubg = rospy.Publisher('gypseas', gypseas, queue_size=10)
    pubd = rospy.Publisher('dolphins', dolphins, queue_size=10)

    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    gyp=gypseas()
    dlp=dolphins()

    gyp.t1=1800
    gyp.t2=1800
    gyp.t3=1800
    gyp.t4=1800
    dlp.d1=1800
    dlp.d2=1800
    dlp.d3=1800
    dlp.d4=1800

    while not rospy.is_shutdown():

        pubg.publish(gyp)
        pubd.publish(dlp)
        rate.sleep()

if __name__ == '__main__':
    try:
      talker()
    except rospy.ROSInterruptException:
        pass