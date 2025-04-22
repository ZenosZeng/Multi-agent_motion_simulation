class Mecanum:
    def __init__(self,dt=0.01):
		# state0=x,state1=y
        self.dt = dt
        self.state = [ 0,0,0 ] # x,y,theta
		
    def reset(self,pos):
        for i in range(len(pos)):
            self.state[i]=pos[i]

    def step(self,u): # u = dx,dy,dtheta
		# kinematic model
        for i in range(len(u)):
            self.state[i] += u[i]*self.dt