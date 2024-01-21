import os
import numpy as np
from BaxterDQN import DQNAgent
from BaxterCustomEnv import BaxterCustomEnv

env = BaxterCustomEnv()

state_size = env.observation_space.shape[0]
action_size = env.action_space.n

n_episodes = 100
n_steps = 100
batch_size = 64
output_dir = 'model_output/baxter/'
if not os.path.exists(output_dir): os.makedirs(output_dir)

agent = DQNAgent(state_size, action_size)

# Interact with environment
q0 = env.baxter.qr
x0 = env.baxter.fkine(q0).t  
t = np.linspace(0, 1, 100)
trajectory = x0 + t[:, np.newaxis] * [-0.4, 0, -0.4]


states = [q0] * (n_episodes - 1)
episode_info = [[]] * (n_episodes - 1)
for j, xd in enumerate(trajectory[1:]):
    done = False
    agent.reset_epsilon()
    env.reset_fitness()
    total_reward = 0
    for e in range(1, n_episodes):
        state = env.reset_state(states[e-1])
        episode_reward = 0

        for t in range(n_steps):
            action = agent.act(state)
            next_state, reward, done, info = env.step(action, xd)
            agent.remember(state, action, reward, next_state, done)
            state = next_state
            
            err = xd - info['x']
            episode_reward += reward
            end='\n' if t == n_steps-1 or done else '\r'

            print(
                f"Episode: {e:^3}, Fitness: {info['fitness']:0.4f}, "
                f"Episode Reward: {episode_reward:+6.0f}, "
                f"Error x: {err[0]:+0.4f}, Error y: {err[1]:+0.4f}, Error z: {err[2]:+0.4f} "
                f"Done: {int(done)}, Epsilon: {agent.epsilon:0.4f}", 
                end=end
            )

            if done:
                break

        episode_info[j].append([e, info['fitness'], episode_reward])

        states[e-1] = next_state
        # agent.remember(state, action, reward, next_state, done)
        if done:
            states[e:] = [next_state] * len(states[e:])
            break
        if len(agent.memory) >= batch_size:
            agent.replay(batch_size)

        total_reward += episode_reward
    # env.render()
    print(
        f"Trajectory point: {j+2}/{len(trajectory)}, "
        f"fitness: {info['fitness']:0.4f}, "
        f"total reward: {total_reward:0.0f}, "
        f"episodes: {e}, epsilon: {agent.epsilon:0.4f}"
    )    


agent.save(output_dir + f"weights.hdf5")

with open(output_dir + 'episode_info.npy', 'wb') as f:
    np.save(f, np.array(episode_info, dtype=object))
