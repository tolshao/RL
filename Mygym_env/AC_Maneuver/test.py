from six_dof_toolbox import Aircract
import numpy as np

time_step = 0.01

def main():
	
	print("testing")
	
	x0 = [0, 0, 0]
	vel0 = [1.0, 2.0, 3.0]
	euler0 = [0.1, 0.2, 0.3]
	ang_vel0 = [0.1, 0.2, 0.3]

	initial_parameters = [x0, vel0, euler0, ang_vel0]
	ac = Aircract(initial_parameters,Tsc = 0.01)
	
	# testing----------------
	print(ac.vel)
	vel = ac.Vel_g_get()
	print(vel)
	
	forces = [1,1,1]
	print(type(forces))
	moments = [1,1,1]
	
	print(ac.vel,"before")
	print(ac.ang_vel,"before")
	ac.update_onestep(forces, moments)
	print(ac.vel)
	print(ac.ang_vel)
	
	
	
if __name__ == "__main__":
	main()