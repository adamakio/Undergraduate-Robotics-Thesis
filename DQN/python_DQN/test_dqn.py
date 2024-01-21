import os
import numpy as np
from BaxterDQN import DQNAgent

from BaxterCustomEnv import BaxterCustomEnv

env = BaxterCustomEnv()

state_size = env.observation_space.shape[0]
action_size = env.action_space.n
output_dir = 'model_output/baxter/'
if not os.path.exists(output_dir): os.makedirs(output_dir)

agent = DQNAgent(state_size, action_size)
agent.load(output_dir + f"weights_850.hdf5")

# Interact with environment
q0 = env.baxter.qr
x0 = env.baxter.fkine(q0).t  
t = np.linspace(0, 1, 200)
trajectory = x0 + t[:, np.newaxis] * [-0.2, -0.2, 0]

states = [q0]
agent.reset_epsilon(0)
state = env.reset_state(states[0])
for xd in trajectory[1:]:
    done = False
    while not done:
        action = agent.act(states[-1])
        next_state, reward, done, _ = env.step(action)
        states.append[next_state]

with open(output_dir + 'states.npy', 'wb') as f:
    np.save(f, np.array(states))