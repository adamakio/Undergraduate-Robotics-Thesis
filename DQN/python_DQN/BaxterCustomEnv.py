import gym
import itertools
import numpy as np
import roboticstoolbox as rtb
import matplotlib.pyplot as plt

from gym import spaces

dq_deg = 0.0025
fg = 0.0015
m = 100

class BaxterCustomEnv(gym.Env):
    """Custom Environment that follows gym interface"""
    metadata = {'render.modes': ['human']}

    def __init__(self):
        super(BaxterCustomEnv, self).__init__()
        # Combinations of actions
        self.combinations = list(itertools.product([-1, 0, 1], repeat=7))
        self.action_space = spaces.Discrete(len(self.combinations))
        # Example for using image as input:
        self.observation_space = spaces.Box(low=-np.pi, high=np.pi,
                                            shape=(7,), dtype=np.float64)
        # Create a robot
        self.baxter = rtb.models.DH.Baxter()
        # Joint angle increment
        self.dq = np.deg2rad(dq_deg)
        # Parameters
        self.goal_fitness = fg
        self.m = m

    def step(self, action, xd): 
        self.baxter.q += self.dq * np.array(self.combinations[action])
        self.x = self.baxter.fkine(self.baxter.q).t
        fitness = np.linalg.norm(self.x-xd)

        reward = self.m * np.arctan(
            0.5 * np.pi * (self.prev_fitness - fitness) / self.goal_fitness
        )  
        self.prev_fitness = fitness 
        
        done = fitness <= self.goal_fitness

        info = {'fitness': fitness, 'x': self.x}
        return self.baxter.q, reward, done, info

    def reset_fitness(self):
        self.prev_fitness = 0
    
    def reset_state(self, q):
        self.baxter.q = q
        return q

    def render(self, mode='human'):
        self.baxter.plot(self.baxter.q, backend='pyplot')
        #plt.plot(*self.x)

    def close (self):
        plt.close()

