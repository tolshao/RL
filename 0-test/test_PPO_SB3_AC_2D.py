import gym

from stable_baselines3 import PPO
from stable_baselines3.ppo import MlpPolicy
from stable_baselines3.common.env_util import make_vec_env
import matplotlib.pyplot as plt
import numpy as np

# Parallel environments
env = make_vec_env('AC_2d-v0', n_envs=1)


model = PPO(MlpPolicy, env, verbose=2)
model.learn(total_timesteps=800000)
model.save("AC_2d_SB3")

# del model # remove to demonstrate saving and loading


# test
model = PPO.load("AC_2d_SB3")


env = gym.make('AC_2d-v0')
obs = env.reset()

print('states:\n', env.reset())
print('action:\n', env.action_space.__repr__())
print('obs:\n', env.observation_space.__repr__())

data = []
rwd_list = []
t = 200
# tsc = 20

for _ in range(int(t / env.tsc)):
    action, _states = model.predict(obs)
#    obs, rewards, dones, info = env.step(action)
    obs, reward, done, _ = env.step(action)
    tmp = np.concatenate((env.pos(), [reward]), axis=0)
    data.append(tmp)
    rwd_list.append(env.rwd_list)
    if done:
        break
# 获取数据plot
data = np.array(data)
rwd_list = np.array(rwd_list)
t = np.linspace(0, t, np.shape(data)[0])
vel = data[:, 0]
phi = data[:, 1]
phi_dot = data[:, 2]
xd = data[:, 3]
yd = data[:, 4]
x = data[:, 5]
y = data[:, 6]
reward = data[:, 7]

plt.figure(figsize=(10, 12))
plt.subplot(3, 2, 1)
plt.plot(x, y, label='position of aircraft')
plt.xlabel('x position')
plt.ylabel('y position')
plt.legend(loc='upper right')

plt.subplot(3, 2, 2)
plt.plot(xd, yd, label='target position')
plt.xlabel('x target')
plt.ylabel('y target')
plt.legend(loc='upper right')

plt.subplot(3, 2, 3)
plt.plot(t, vel, label='velocity')
plt.xlabel('time')
plt.ylabel('aircraft velocity')
plt.legend(loc='upper right')

plt.subplot(3, 2, 4)
plt.plot(t, phi_dot, label='$\omega_{phi}$')
plt.xlabel('time')
plt.ylabel('angular velocity')
plt.legend(loc='upper right')

plt.subplot(3, 2, 5)
plt.plot(t, phi/np.pi*180, label='angle')
plt.xlabel('time')
plt.ylabel('angle ($\theta$)')
plt.legend(loc='upper right')

plt.subplot(3, 2, 6)
plt.plot(t, rwd_list[:, 0], 'b-', label='distance')
plt.plot(t, rwd_list[:, 1], 'g', label='vel')
plt.plot(t, rwd_list[:, 2], 'r', label='angle')
plt.plot(t, reward, 'black', label='total')
plt.xlabel('time')
plt.ylabel('Reward')
plt.legend(loc='upper right')

plt.savefig('test.pdf')
# plt.show()
