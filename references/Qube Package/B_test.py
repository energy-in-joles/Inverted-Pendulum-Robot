import gym
import quanser_robots

env = gym.make('Qube-500')
env.reset()
env.render()
