from math import cos,sin

class Unicycle:
    def __init__(self,dt=0.01):
		# state0=x,state1=y
        self.dt = dt
        self.state = [ 0 for i in range(3)] # x y theta
		
    def reset(self,pos):
        for i in range(len(pos)):
            self.state[i]=pos[i]

    def step(self,u):
		# kinematic model
        '''
        dot_x = cos(theta)*v
        dot_y = sin(theta)*v
        dot_theta = w
        '''
        v,w,theta = u[0],u[1],self.state[2]
        dot_x = cos(theta)*v
        dot_y = sin(theta)*v
   
        self.state[0] += dot_x*self.dt
        self.state[1] += dot_y*self.dt
        self.state[2] += w*self.dt	

    def output_pos(self,offset):
        x = self.state[0] + offset*cos(self.state[2])
        y = self.state[1] + offset*sin(self.state[2])
        return [x,y] # pos list