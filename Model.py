import numpy as np
from math import cos,sin,atan

class Akman_Dynamic:
	def __init__(self,M=2,R=0.02,L=0.1,J=0.02,x=0.0,y=0.0,theta=0.0,v=0.0,w=0.0,T_control=0.1):
		self.x = x
		self.y = y
		self.theta = theta
		self.v = v
		self.w = w
		self.T_control = T_control
		self.dt = self.T_control/10
		self.R = R
		self.M = M
		self.L = L
		self.J = J
		# self.energy = 0
		
	def reset(self,x,y,theta,v=0,w=0):
		self.x = x
		self.y = y
		self.theta = theta
		self.v = v
		self.w = w
		# self.energy = 0

	def step(self,ur,ul):
		# Dynamic model
		# ur ul -> ax ay atheta
		# solve a_theta
		atheta = (ur-ul)* self.L / self.J / self.R
		# solve ax ay AX=B
		a = (  (ur+ul)/(self.M*self.R)  )

		# a atheta -> v w
		# self.v += a*self.dt
		self.w += atheta*self.dt
		
		ax = a*cos(self.theta)-self.v*sin(self.theta)*self.w
		ay = a*sin(self.theta)+self.v*cos(self.theta)*self.w
	
		vx = self.v*cos(self.theta)+ax*self.dt
		vy = self.v*sin(self.theta)+ay*self.dt

		self.v=(vx**2+vy**2)**0.5
		self.theta += self.w*self.dt
		
		# v w -> x y theta
		self.x += vx*self.dt
		self.y += vy*self.dt
		
	def run(self,ur,ul):
		for i in range(10):
			self.step(ur,ul)



class Single_Integrator:
    def __init__(self,dimension=2,dt=0.01):
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
