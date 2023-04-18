import numpy as np
from scipy.integrate import trapezoid
from tf.transformations import euler_from_quaternion


class pid:

    def __init__ (self):
      
        self.k=[]
        self.pid_i=0
        self.error=[0]
        self.time=[0]
        self.current_vel=0
        self.prev_vel=0
        self.current_position=0
        self.final=0
        self.time_elapsed=0
        # only for time being for linear motion - when we know how much distance is left to be travelled, then we can remove this part
        # if we are integrating for linear motion:
        # self.acc = []
        # self.vel = []
    
    def integrate(self, y, x):
        return trapezoid(y, x)
    
    def convert(x,y,z,w):

        orientation_list = [x,y,z,w]

        (roll, pitch, yaw) = np.degrees(euler_from_quaternion (orientation_list))

        return roll, pitch, yaw
    
    def getPID(self,feed_forward=False):
    
        kp=self.k[0];ki=self.k[1];kd=self.k[2]
        
        current=self.current_position
        error = self.final - current
        # this 'self.value' will be the ros subscriber cam feed - the distance that is left to travel.
        # error = self.final
        self.error.append(error)
        self.time.append(self.time_elapsed)

        pid_i = self.integrate(self.error, self.time)
        self.pid_i=pid_i

        if(feed_forward):
            feedforward = self.current_vel - self.prev_vel
        else:
            feedforward=0
        
        pid_p = kp*error
        pid_d = kd*(error-self.error[-1]) 

        if pid_i>max(90-pid_p-pid_d, 0):
            pid_i = max(90-pid_p-pid_d,0)
        
        elif pid_i<min(-90-pid_i-pid_d, 0):
            pid_i = min(-90-pid_p-pid_d,0)

        pid_i_final = ki*pid_i

        if self.time_elapsed>0:
            PID = pid_p + pid_i_final + pid_d + feedforward/self.time_elapsed
        else:
            PID = pid_p + pid_i_final + pid_d
        self.prev_vel = self.current_vel
        
        if(PID > 75):
            PID=75
        if(PID < -75):
            PID=-75
        
        return PID