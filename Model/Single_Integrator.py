class Single_Integrator:
    def __init__(self,dimension,dt=0.01):
		# state0=x,state1=y
        self.dt = dt
        self.state = [ 0 for i in range(dimension)]
		
    def reset(self,pos):
        for i in range(len(pos)):
            self.state[i]=pos[i]

    def step(self,u):
		# kinematic model
        for i in range(len(u)):
            self.state[i] += u[i]*self.dt