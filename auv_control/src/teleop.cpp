#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include <map>
using namespace ros;

// /auv/BLDC_1_motor_controller/command
// /auv/BLDC_2_motor_controller/command
// /auv/BLDC_3_motor_controller/command
// /auv/BLDC_4_motor_controller/command
// /auv/BLDC_5_motor_controller/command
// /auv/BLDC_6_motor_controller/command
// /auv/BLDC_7_motor_controller/command
// /auv/BLDC_8_motor_controller/command

// 1:left upper 
// 2:right upper
// 3:right lower
// 4:left lower
// 5:front left
// 6:front right
// 7:back right
// 8:back left

float BLDC_1=0.0;
float BLDC_2=0.0;
float BLDC_3=0.0;
float BLDC_4=0.0;
float BLDC_5=0.0;
float BLDC_6=0.0;
float BLDC_7=0.0;
float BLDC_8=0.0;
float MIN_THRUST=0.0;
float MAX_THRUST=3751;
float LIFT=0;
float gain=2;


Publisher PBLDC_1;
Publisher PBLDC_2;
Publisher PBLDC_3;
Publisher PBLDC_4;
Publisher PBLDC_5;
Publisher PBLDC_6;
Publisher PBLDC_7;
Publisher PBLDC_8;

char key(' ');

float check(float x)
{
    
    if(x<=0)
    {
        return 0;
    }
    
    if (x>=MAX_THRUST)
    {
        return MAX_THRUST;
    }
    return x;
}

class movements
{
    public:
    void PublishV()
    {
        std_msgs::Float64 temp;
        
        temp.data=check(BLDC_1);
        PBLDC_1.publish(temp);

        temp.data=check(BLDC_2);
        PBLDC_2.publish(temp); 

        temp.data=check(BLDC_3);
        PBLDC_3.publish(temp);

        temp.data=check(BLDC_4);
        PBLDC_4.publish(temp);

        temp.data=check(BLDC_5);
        PBLDC_5.publish(temp);

        temp.data=check(BLDC_6);
        PBLDC_6.publish(temp);

        temp.data=check(BLDC_7);
        PBLDC_7.publish(temp);

        temp.data=check(BLDC_8);
        PBLDC_8.publish(temp);


    }

    void reset_z()
    {
        BLDC_1=LIFT;
        BLDC_2=LIFT;
        BLDC_3=LIFT;
        BLDC_4=LIFT;
        PublishV();
    }

    void ZThrust(float x)
    {
        float y=x*gain;
        LIFT+=y;
        LIFT+=y;
        LIFT+=y;
        LIFT+=y;
        BLDC_1=check(LIFT);
        BLDC_2=check(LIFT);
        BLDC_3=check(LIFT);
        BLDC_4=check(LIFT);

        PublishV();
    }

    void Pitch(float x)
    {
        float y=x*gain;
        
        if(x>0)
        {
            reset_z();
            BLDC_1+=y;
            BLDC_2+=y;
        }

        else
        {
            reset_z();
            BLDC_3+=abs(y);
            BLDC_4+=abs(y);
        }

        PublishV();
    }

    void Roll(float x)
    {
        float y=x*gain;
        if(x>0)
        {
            BLDC_2+=y;
            BLDC_3+=y;
        }
        else
        {
            BLDC_1+=abs(y);
            BLDC_4+=abs(y);
        }
        PublishV();
    }

    void Front_Back(float x)
    {
        float y=x*gain;
        if(x>0)
        {
            BLDC_5=0;
            BLDC_6=0;
            BLDC_7+=y;
            BLDC_8+=y;
        }
        else
        {
            BLDC_7=0;
            BLDC_8=0;
            BLDC_5+=abs(y);
            BLDC_6+=abs(y);
        }
        PublishV();
    }
    
    void left_Right(float x) 
    {
        float y=x*gain;
        if(x>0)
        {
            BLDC_5=0;
            BLDC_7=0;
            BLDC_8+=abs(y);
            BLDC_6+=abs(y);
        }
        else
        {   
            BLDC_5+=abs(y);
            BLDC_7+=abs(y);
            BLDC_8=0;
            BLDC_6=0;
        }
    }   
    void reset()
    {
        BLDC_1=LIFT;
        BLDC_2=LIFT;
        BLDC_3=LIFT;
        BLDC_4=LIFT;
        BLDC_5=0;
        BLDC_6=0;
        BLDC_7=0;
        BLDC_8=0;
        PublishV();
    }

};

std::string msg = R"(

Reading from the keyboard and Publishing to Auv!
---------------------------
Moving around:
   w :move front
   s :move back
   a :left
   d :right
   i :pitch up
   k :pitch down
   j :yaw left
   k :yaw righ
   y :move up
   h :move down
   r :reset

CTRL-C to quit

)";


int getch(void)
{
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}


int main(int argc, char **argv)
{
    init(argc, argv, "teleop_node");

    NodeHandle n;
    Rate loop_rate(5);

    std::cout<<msg;

    PBLDC_1 = n.advertise<std_msgs::Float64>("/thruster_1_controller/command", 1000);
    PBLDC_2 = n.advertise<std_msgs::Float64>("/thruster_2_controller/command", 1000);
    PBLDC_3 = n.advertise<std_msgs::Float64>("/thruster_3_controller/command", 1000);
    PBLDC_4 = n.advertise<std_msgs::Float64>("/thruster_4_controller/command", 1000);
    PBLDC_5 = n.advertise<std_msgs::Float64>("/thruster_5_controller/command", 1000);
    PBLDC_6 = n.advertise<std_msgs::Float64>("/thruster_6_controller/command", 1000);
    PBLDC_7 = n.advertise<std_msgs::Float64>("/thruster_7_controller/command", 1000);
    PBLDC_8 = n.advertise<std_msgs::Float64>("/thruster_8_controller/command", 1000);
    movements auv;

    while (true)
    {
        int c = getch();

        if (c == 'y')
        {
            auv.ZThrust(1);
        }

        else if (c == 'h')
        {
            auv.ZThrust(-1);
        }
        else if (c == 'i')
        {
            auv.Pitch(1);   
        }
        else if (c == 'k')
        {
            auv.Pitch(-1);
        }
        else if (c == 'l')
        {
            auv.Roll(1);
        }
        else if (c == 'j')
        { 
            auv.Roll(-1);
        }
        else if (c == '\x03')
        {
            printf("\n\n........ ABORTING!!!! .......... ABORTING!!!! ........ \n\n");
            break;
        }
        else if (c == 'a')
        { 
            auv.left_Right(1);
        }
        else if (c == 'd')
        { 
            auv.left_Right(-1);
        }
        else if (c == 'w')
        { 
            auv.Front_Back(1);
        }
        else if (c == 's')
        { 
            auv.Front_Back(-1);
        }
         else if (c == 'r')
        { 
            auv.reset();
        }

        spinOnce();
        loop_rate.sleep();
    }
}