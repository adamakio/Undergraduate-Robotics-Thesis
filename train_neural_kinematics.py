import autograd.numpy as anp
from autograd import jacobian

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt 

from sklearn.neural_network import MLPRegressor
from sklearn.metrics import mean_squared_error
from sklearn.model_selection import train_test_split

import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense

def read_data():
    # Load the input file (q_ds.csv) into a pandas DataFrame
    input_df = pd.read_csv('data/q_ds.csv')

    # Load the output file (x_ds.csv) into a pandas DataFrame
    output_df = pd.read_csv('data/x_ds.csv')

    # Convert the DataFrames to arrays
    input_array = input_df.values
    output_array = output_df.values
    return input_array, output_array

def split_data(input_array, output_array):
    # Return the input and output arrays split into training and test sets
    return train_test_split(
        input_array, output_array, test_size=0.2, random_state=0
    )
    
def train_MLP_on_full_dataset(input_array, output_array):
    # Train the multi-layer perceptron using tensorflow
    mlp = Sequential([
        Dense(input_shape=(7,), units=256, activation='tanh'),  
        Dense(128, activation='tanh'), 
        Dense(3, activation='linear'),  
    ])
    
    mlp.compile(
        optimizer='adam', loss='mse', metrics=['accuracy']
    )
    mlp.fit(
        input_array, output_array, 
        epochs=100, 
        batch_size=1000, 
        validation_split=0.2
    )
    mlp.save('saved_models/neural_kinematics')
    return mlp

def train_MLP(input_train, output_train):
    # Train the multi-layer perceptron using tensorflow
    mlp = Sequential([
        Dense(input_shape=(7,), units=256, activation='tanh'),  
        Dense(128, activation='tanh'), 
        Dense(3, activation='linear'),  
    ])
    
    mlp.compile(
        optimizer='adam', loss='mse', metrics=['accuracy']
    )
    mlp.fit(
        input_train, output_train, 
        epochs=100, 
        batch_size=1000, 
        validation_split=0.2
    )
    mlp.save('saved_models/neural_kinematics')
    return mlp

def evaluate_MLP(mlp, input_test, output_test):
    # Evaluate the performance of the model on the test set
    test_score = mlp.score(input_test, output_test)
    # Evaluate the model using mean squared error
    predictions = mlp.predict(input_test)
    mse = mean_squared_error(output_test, predictions)
    return test_score, mse

def plot_predictions(predictions, output_test):
    predictions = np.reshape(predictions, (-1, 3, 7))
    output_test = np.reshape(output_test, (-1, 3, 7))
    for i in range(1, 4):
        for j in range(1, 8):
            # Plot the results or history
            J_hat_ij = predictions[:, i - 1, j - 1]
            J_ij = output_test[:, i - 1, j - 1]
            plt.figure()
            plt.plot(J_ij, label='Actual Jacobian')
            plt.plot(J_hat_ij, label='Estimated Jacobian')
            plt.xlabel('Sample #')
            plt.ylabel(f'Jacobian')
            plt.legend()
            plt.savefig(f'python_plots/jacobian/J_{i}{j}.png')
    
def plot_error(predictions, output_test):
    predictions = np.reshape(predictions, (-1, 3, 7))
    output_test = np.reshape(output_test, (-1, 3, 7))
    error_norm = np.linalg.norm(predictions - output_test, 'fro', axis=(1, 2))
    reshaped_error_norm = np.reshape(np.outer(error_norm, np.ones((21,))), (-1, 3, 7))
    error = np.divide(np.abs(predictions - output_test), reshaped_error_norm)
    for i in range(1, 4):
        for j in range(1, 8):
            # Plot the results or history
            eij = error[:, i - 1, j - 1]
            plt.figure()
            plt.hist(eij, bins=50)
            plt.ylabel('Histogram frequency')
            plt.xlabel(f'Normalized error at ({i}, {j})')
            plt.savefig(f'python_plots/error/e_{i}{j}.png')

def predict(mlp, q):
    return mlp.predict(q)

def is_jacobian_correct(jacobian_fn, ffpass_fn, n_inputs=7):
    """ Check of the Jacobian using numerical differentiation
    """
    q = np.random.random((n_inputs,))
    epsilon = 1e-5
    """ Check a few columns at random
    """
    for idx in np.random.choice(n_inputs, 3, replace=False):
        q2 = q.copy()
        q2[idx] += epsilon
        num_jacobian = (ffpass_fn(q2) - ffpass_fn(q)) / epsilon
        computed_jacobian = jacobian_fn(q)
        
        if not all(abs(computed_jacobian[:, idx] - num_jacobian) < 1e-3): 
            return False
    return True

def ffpass_tf(mlp, q):
    """ The feedforward function of our neural net
    """    
    qr = q.reshape((1, q.size))
    return mlp.predict(qr)[0]


def ffpass_anp(mlp, q):
    # Get weights
    w1 = mlp.layers[0].weights.numpy()
    b1 = mlp.layers[0].bias.numpy()
    w2 = mlp.layers[1].weights.numpy()
    b2 = mlp.layers[1].bias.numpy()
    w3 = mlp.layers[2].weights.numpy()
    b3 = mlp.layers[2].bias.numpy()

    # Compute output
    a1 = anp.dot(q, w1) + b1   # affine
    a1 = anp.tanh(a1)    # tanh
    a2 = anp.dot(a1, w2) + b2  # affine
    a2 = anp.tanh(a2)    # tanh
    a3 = anp.dot(a2, w3) + b3  # affine
    out = a3
    return out

def jacobian_autograd(x):
    return jacobian(ffpass_anp)(x)

if __name__ == '__main__':
    train = 0 # 0 to save 1 to train 
    if not train:
        input_array, output_array = read_data()
        print('Data read.')
        train_MLP_on_full_dataset(input_array, output_array)
        print('Model saved.')
    else:
        input_array, output_array = read_data()
        print('Data read.')
        input_train, input_test, output_train, output_test = split_data(input_array, output_array)
        print('Data split.')
        mlp = train_MLP(input_train, output_train)
        print('Model trained.')
        test_score, mse = evaluate_MLP(mlp, input_test, output_test)
        print('Model evaluated.')
        # Print the test score
        with open('python_plots/scores.txt', 'w') as f:
            f.write(f'Test score: {test_score}\n')
            f.write(f'Test Mean Squared Error: {mse}')
        
        # Get predictions
        predictions = predict(mlp, input_test)
        print('Predictions made.')
        plot_predictions(predictions, output_test)
        print('Predictions plotted')
        plot_error(predictions, output_test)
        print('Error plotted.')
