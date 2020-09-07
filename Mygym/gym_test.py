import gym
import numpy as np
import matplotlib.pyplot as plt


env = gym.make('AC_brake-v0')

env.reset()

for i in range(300):
    obs, reward, done, _ = env.step(1)
    data = [obs, reward, done]
    # print(obs)
    if done:
        print(i)
        break
print(int(obs[1]), reward)
