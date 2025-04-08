import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from stable_baselines3 import PPO
from src.sim.quadson_env import QuadsonEnv, PlottingCallback
env = QuadsonEnv()
# model = PPO("MlpPolicy", env, verbose=1)
model = PPO('MlpPolicy', env, verbose=1, learning_rate=5e-5, n_steps=2048, batch_size=256)
plotting_callback = PlottingCallback()
model.learn(total_timesteps=4000000, callback=plotting_callback)
model.save("quadson_ppo")