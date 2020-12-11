import gym
env = gym.make('CartPole-v0')
env.reset()
for t in range(1000):
    env.render()
    obs, reward,done,_ = env.step(env.action_space.sample()) # take a random action
    print(t,obs)
    if done:
        env.close()
        break
        
