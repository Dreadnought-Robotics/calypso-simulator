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
        self.acc = [0]
        self.vel = [0]
    
    def floor_thruster(self,x,val):
        if x<val:
            return val
        else :
            return x
    
    def interpolate(self,s,a,b):

        return b[0]+((s-a[0])*(b[1]-b[0])/(a[1]-a[0]))
        
    
    def integrate(self, y, x):
        try:
            return trapezoid(y, x)
        except Exception as e:
            print(len(y),len(x))
            print("Sync failure !!")
    
    def convert(x,y,z,w):

        orientation_list = [x,y,z,w]

        (roll, pitch, yaw) = np.degrees(euler_from_quaternion (orientation_list))

        return roll, pitch, yaw
    
    def get_current_pose(self):
        
        print("pose")
        self.vel.append(self.integrate(self.acc,self.time))
        return self.integrate(self.vel,self.time)

    def getPID(self,feed_forward=False):
    
        kp=self.k[0];ki=self.k[1];kd=self.k[2]
        pid_i=0
        pid_p=0
        pid_d=0
        
        current=self.current_position
        error = self.final - current
        
        # this 'self.value' will be the ros subscriber cam feed - the distance that is left to travel.
        # error = self.final
        
        self.error.append(error)

        pid_i = self.integrate(self.error, self.time)
        
        self.pid_i=pid_i
        pid_p = kp*error

        if(feed_forward):
            feedforward = self.current_vel - self.prev_vel
        else:
            feedforward=0
        
        try:
            pid_d = kd*(error[-1]-self.error[-2])/(self.time[-2]-self.time[-1]) 
        except:
            pass

        if pid_i>max(90-pid_p-pid_d, 0):
            pid_i = max(90-pid_p-pid_d,0)
        
        elif pid_i<min(-90-pid_i-pid_d, 0):
            pid_i = min(-90-pid_p-pid_d,0)

        pid_i_final = ki*pid_i

        time_elapsed=self.time[-1]

        if time_elapsed>0:
            PID = pid_p + pid_i_final + pid_d + feedforward/time_elapsed
        else:
            PID = pid_p + pid_i_final + pid_d
        self.prev_vel = self.current_vel
        
        if(PID > 75):
            PID=75
        if(PID < -75):
            PID=-75
        
        return PID