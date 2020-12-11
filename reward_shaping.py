import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
# %matplotlib inline
import cv2

def r_angle(x, y):
	return -50 * np.arctan((np.abs(y/x)+0.1) ** 1.2 ) # * (np.abs(x)>10) + (- 0.01 * y **2) * (np.abs(x)<=10)
	# if np.abs(x)<10 else -x **2
	



x = np.linspace(-100, 100, 500)
y = np.linspace(-50, 50, 500)
X, Y = np.meshgrid(x, y)
Z = r_angle(X, Y)

fig = plt.figure(figsize = (6,8))

#print(Z.shape)
#Z = cv2.blur(Z,(50,50))

# fig 1
ax1 = fig.add_subplot(211,projection='3d')  #这种方法也可以画多个子图
ax1.plot_surface(X, Y, Z, rstride=10, cstride=10,
				cmap='viridis', edgecolor='none')

plt.xlabel('x')
plt.ylabel('y')
ax1.set_zlabel('Reward_angle')
#ax.set_ylabel('y')
#ax.set_zlabel('z')
# 调整观察角度和方位角。这里将俯仰角设为60度，把方位角调整为35度
ax1.view_init(60, 35)

ax1.set_title('Reward of deviation ')
#plt.show()




# figure 2 Velocity Reward



fig.add_subplot(223)
plt.plot(x,y)

plt.title('Velocity Reward ')

# figure 3 Distance Reward


fig.add_subplot(224)
plt.plot(x,y)
plt.title('Distance Reward')

plt.show()
