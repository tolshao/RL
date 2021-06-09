import gym
from scipy.io import savemat
import time
from stable_baselines3 import PPO
from stable_baselines3.ppo import MlpPolicy, CnnPolicy
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.evaluation import evaluate_policy

# Parallel environments

# cartpole game
# env_name = 'CartPole-v0'
# policy_name = 'MlpPolicy'

# CarRacing game
env_name = 'CarRacing-v0'
policy_name = 'CnnPolicy'

env = make_vec_env(env_name, n_envs=2)

model = PPO(policy_name, env, verbose=0)
# model = PPO.load("weights/{}_{}_sb3.zip".format(env_name, policy_name))
# model.set_env(env)
start_time = time.time()
model.learn(int(3e4))
stop_time = time.time()
print(f"time spent is {stop_time-start_time}s")
model.save("weights/{}_{}_sb3.zip".format(env_name, policy_name))

# del model  # remove to demonstrate saving and loading

# evaluate the agent
model = PPO.load("weights/{}_{}_sb3.zip".format(env_name, policy_name))
eval_env = gym.make(env_name)
mean_reward, std_reward = evaluate_policy(model, eval_env, n_eval_episodes=10)
print(f"mean reward: {mean_reward:.2f} +- {std_reward:.2f}")

obs = env.reset()
Total_reward = 0
for i in range(50*3):
    action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)
    env.render()
    Total_reward += rewards

print(Total_reward)
