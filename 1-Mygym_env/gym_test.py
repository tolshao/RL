import gym
import numpy as np
import matplotlib.pyplot as plt
plt.style.use(['science'])

env = gym.make('AC_brake-v0')

env.reset()

data_list = []
endtime = 0
for i in range(300):
    obs, reward, done, _ = env.step(1)
    data_list.append([obs, reward, done])
    # print(obs)
    if done:
        print('The episode ended at {} steps'.format(i + 1))
        endtime = i
        break


# plot data_list

time = np.arange(endtime+1)
data_list = np.array(data_list)
print(np.shape(data_list))
yy = []
for data in data_list[:, 0]:
    yy.append(data[0])
print('a')
print('b')
plt.plot(time, yy)
plt.show()
