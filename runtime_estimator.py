from sklearn.neural_network import MLPRegressor
import numpy as np
import time

import scitime

# example for rf regressor
estimator = scitime.estimate.Estimator(meta_algo='NN', verbose=3)
mlp = MLPRegressor(hidden_layer_sizes=(256, 128), max_iter=3000, random_state=0, verbose=True, early_stopping=True)

n_samples = 10_000_000
X,y = np.random.rand(n_samples,14),np.random.rand(n_samples,21)
# run the estimation
estimation, lower_bound, upper_bound = estimator.time(mlp, X, y)

print(f'{estimation=}, {lower_bound=}, {upper_bound=}')