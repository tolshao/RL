from Aircraft import Aircraft
import numpy as np

time_step = 0.01

def test():
	
	print("testing----------------------------------------------------")
	
	x0 = [0.0, 0.0, 0.0]
	vel0 = [1.0, 2.0, 3.0]
	euler0 = [0.1, 0.2, 0.3]
	ang_vel0 = [0.1, 0.2, 0.3]

	initial_parameters = [x0, vel0, euler0, ang_vel0]
	ac = Aircraft(initial_parameters,Tsc = 0.01,RK4 = False)
	forces = [1,1,1]
	forces = [0,0,0]
	moments = [0,0,0]
	print(ac.RK4)
	print(ac.vel,ac.ang_vel)
	print(ac.DotAngVel_get())
	ac.update_onestep(forces, moments)
	print(ac.vel,ac.ang_vel)
	
	
	
if __name__ == "__main__":
	test()