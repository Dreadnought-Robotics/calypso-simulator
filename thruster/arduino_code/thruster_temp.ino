#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Empty.h>
#include <thruster/gypseas.h>
#include <Servo.h>

ros::NodeHandle  nh;

Servo esc1;

#define upper_limit 2000
#define lower_limit 1000
#define stop_sig 1500

thruster::gypseas a;

void apply_thruster(const thruster::gypseas& t){
  esc1.writeMicroseconds(t.t1);
}

ros::Subscriber<planner::gypseas> sub("thruster_test", apply_thruster );



std_msgs::String str_msg;
ros::Publisher chatter("auv_publisher", &str_msg);

char hello[13] = "hello world!";

void setup()
{
  esc1.attach(6,lower_limit,upper_limit);
  esc1.writeMicroseconds(stop_sig);
  delay(3000);
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(500);
}
