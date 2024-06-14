# export LD_LIBRARY_PATH=/opt/miniconda3/envs/rlgpu/lib 
import isaacgym
import isaacgymenvs
import torch
num_envs = 1

import os
os.environ["CUDA_VISIBLE_DEVICES"] = "0"	

envs = isaacgymenvs.make(
	seed=0, 
	task="Ant", 
	num_envs=num_envs, 	
	sim_device="cuda:0",
	rl_device="cuda:0",
)
# print("Observation space is", envs.observation_space)
# print("Action space is", envs.action_space)
# obs = envs.reset()
# for _ in range(20):
# 	random_actions = 2.0 * torch.rand((num_envs,) + envs.action_space.shape, device = 'cpu	') - 1.0
# 	envs.step(random_actions)					