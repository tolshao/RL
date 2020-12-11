import gym

from stable_baselines3 import PPO
from stable_baselines3.ppo import MlpPolicy, CnnPolicy
from stable_baselines3.common.env_util import make_vec_env

# Parallel environments

# cartpole game
env_name = 'CartPole-v0'
policy_name = 'MlpPolicy'

# CarRacing game
env_name = 'CarRacing-v0'
policy_name = 'CnnPolicy'

env = make_vec_env(env_name, n_envs=4)

model = PPO(policy_name, env, verbose=4)
model = PPO.load("models/{}_{}_sb3.zip".format(env_name, policy_name))
# model.learn(total_timesteps=2e4)
# model.save("models/{}_{}_sb3.zip".format(env_name, policy_name))

# del model  # remove to demonstrate saving and loading

# model = PPO.load("models/{}_{}_sb3.zip".format(env_name, policy_name))

obs = env.reset()
while True:
    action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)
    env.render()
