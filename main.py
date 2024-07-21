import gym


env = gym.make("CartPole-v1", render_mode="rgb_array")
env.reset()
while True:
    env.render()