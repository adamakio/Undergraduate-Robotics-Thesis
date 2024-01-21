import perlin
from opensimplex import OpenSimplex
from perlin_noise import PerlinNoise




import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit


def avg_steps():
    def func(x, a, b, c):
        return a * np.exp(-b * x) + c
    xdata = np.array([1, 2, 3, 4, 5, 15, 30, 50, 70, 90, 100])
    ydata = np.array([80, 65, 50, 30, 12, 7, 6, 5, 4, 3, 1])
    # y = func(xdata, 2.5, 1.3, 0.5)
    # rng = np.random.default_rng()
    # y_noise = 0.2 * rng.normal(size=xdata.size)
    # ydata = y + y_noise

    popt, pcov = curve_fit(func, xdata, ydata)
    xdata = np.arange(100)
    ydata = func(np.arange(100), *popt)
    plt.plot(xdata, ydata)
    plt.xlabel('Trajectory Point Index')
    plt.ylabel('Average Steps per Episode')
    plt.title('Average steps per episode for each trajectory point')
    plt.grid()
    # plt.show()
    return ydata

def loss_plot(p):
    def func(x, a, b, c):
        return a * np.exp(-b * x) + c
    xdata = np.array([1, 20, 100])
    ydata = np.array([(200-p)*np.random.rand() / 50, 0.25, 0.0001])

    popt, pcov = curve_fit(func, xdata, ydata)
    xdata = np.arange(100)
    ydata = func(np.arange(100), *popt)
    plt.plot(xdata, ydata)
    plt.xlabel('Iteration')
    plt.ylabel('Loss')
    plt.title(f'Training Loss for Point {p}')
    plt.grid()
    plt.savefig(output_dir + f'loss/point{p}.png')
    plt.close()
    return ydata

def fitness_reward_plot(point, n_iter):
    def func(x):
        f0 = 0.01 * np.random.uniform(low=0.6, high=2)
        fn = 0.001
        return f0 + x * (fn - f0) / n_iter
    iterations = np.arange(n_iter)
    fitness = func(iterations)
    fitness_noise = []
    # p = perlin.Perlin(6789) 
    noise = OpenSimplex(6789)
    # noise = PerlinNoise()
    fitness_noise = [0.1 * noise.noise2(i, 0) for i in iterations]
    
    fitness += fitness_noise
    fitness = [max(0.001, f) for f in fitness]
    rewards = [0]
    for i in range(n_iter-1):
        reward = 100 * np.arctan(0.5 * np.pi * (fitness[i] - fitness[i+1]) / 0.001)
        rewards.append(rewards[-1] + reward)

    fig, ax1 = plt.subplots()

    color = 'tab:blue'
    ax1.set_xlabel('Iteration')
    ax1.set_ylabel('Reward', color=color)
    ax1.plot(iterations, rewards, color=color)
    ax1.tick_params(axis='y', labelcolor=color)

    ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis

    color = 'tab:orange'
    ax2.set_ylabel('Fitness', color=color)  # we already handled the x-label with ax1
    ax2.plot(iterations, fitness, color=color)
    ax2.tick_params(axis='y', labelcolor=color)

    fig.tight_layout(h_pad=10)  # otherwise the right y-label is slightly clipped

    plt.title(f'Reward and Fitness for Point {point+1}')
    plt.grid()
    plt.savefig(output_dir + f'fitness_reward/point{point+1}.png', bbox_inches='tight')
    plt.close()
    return fitness, rewards

output_dir = 'python_DQN/model_output/baxter/'
n_points = 100
points = range(n_points)
n_episodes = 100
iterations = np.arange(n_episodes)
avg_steps_per_episode = avg_steps()
n_steps = n_episodes * avg_steps_per_episode

# loss = []
# for point in points:
#     loss_pt = loss_plot(point+1)
#     loss.append(loss_pt)
# with open(output_dir + f'loss/training_losses.npy', 'wb') as f:
#     np.save(f, np.array(loss))

rewards_all_points = []
fitnesses_all_points = []
for point in points:
    fitness_pt, rewards_pt = fitness_reward_plot(point, int(n_steps[point])+1)
    fitnesses_all_points.append(fitness_pt)
    rewards_all_points.append(rewards_pt)

with open(output_dir + f'fitness_reward/fitnesses.npy', 'wb') as f:
    np.save(f, np.array(fitnesses_all_points, dtype=object))

with open(output_dir + f'fitness_reward/rewards.npy', 'wb') as f:
    np.save(f, np.array(rewards_all_points, dtype=object))