# Cartpole DQN

import os
import gym
import random
import numpy as np
from collections import deque
from keras.models import Sequential
from keras.layers import Dense
from keras.optimizers import Adam

env = gym.make('CartPole-v0')
state_size = env.observation_space.shape[0]
action_size = env.action_space.n
gamma = 0.95
epsilon = 0
epsilon_decay = 0.995
epsilon_min = 0.01
learning_rate = 0.001
batch_size = 32
n_episodes = 1001
n_steps = 5000
memory_size = 2000
output_dir = 'model_output/cartpole/'
if not os.path.exists(output_dir): os.makedirs(output_dir)

class DQNAgent:
    def __init__(self, state_size, action_size):
        self.state_size = state_size
        self.action_size = action_size

        self.memory = deque(maxlen=memory_size)
        self.gamma = gamma
        self.epsilon = epsilon
        self.epsilon_decay = epsilon_decay
        self.epsilon_min = epsilon_min
        self.learning_rate = learning_rate

        self.model = self._build_model()
        
    def _build_model(self):
        model = Sequential()
        model.add(Dense(24, input_dim=self.state_size, activation='relu'))
        model.add(Dense(24, activation='relu'))
        model.add(Dense(self.action_size, activation='linear'))
        model.compile(loss='mse', optimizer=Adam(learning_rate=self.learning_rate))
        return model
    
    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    def act(self, state):
        if np.random.rand() <= self.epsilon:
            return random.randrange(self.action_size)
        act_values = self.model.predict(state, verbose=0)
        return np.argmax(act_values[0])
    
    def replay(self, batch_size):
        minibatch = random.sample(self.memory, batch_size)
        for state, action, reward, next_state, done in minibatch:
            target = reward
            if not done:
                target += self.gamma * np.amax(self.model.predict(next_state, verbose=0)[0])
            target_f = self.model.predict(state, verbose=0)
            target_f[0][action] = target

            self.model.fit(state, target_f, epochs=1, verbose=0)
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

    def save(self, name):
        self.model.save_weights(name)
    
    def load(self, name):
        self.model.load_weights(name)

agent = DQNAgent(state_size, action_size)
agent.load(output_dir + f"weights_850.hdf5")
state = env.reset()
state = np.reshape(state, [1, state_size])
for t in range(2000):
        env.render()
        action = agent.act(state)
        next_state, reward, done, _ = env.step(action)

        reward = reward if not done else -1 
        next_state = np.reshape(next_state, [1, state_size])
        # agent.remember(state, action, reward, next_state, done)
        state = next_state
"""
# Interact with environment
done = False
for e in range(n_episodes):
    state = env.reset()
    state = np.reshape(state, [1, state_size])
    for t in range(n_steps):
        # env.render()
        action = agent.act(state)
        next_state, reward, done, _ = env.step(action)

        reward = reward if not done else -1 
        next_state = np.reshape(next_state, [1, state_size])
        agent.remember(state, action, reward, next_state, done)
        state = next_state
        if done:
            print(f"Episode: {e}/{n_episodes}, score {t}, epsilon: {agent.epsilon}")
            break

    if len(agent.memory) >= batch_size:
        agent.replay(batch_size)
        
    if e % 50 == 0:
        agent.save(output_dir + f"weights_{e}.hdf5")
"""