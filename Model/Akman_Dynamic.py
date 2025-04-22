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




           

		   
