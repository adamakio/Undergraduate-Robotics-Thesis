""" 
Neural Jacobian Python Script

Author: Zouhair Adam Hamaimou
Date: 2023-02-12
"""


import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from sklearn.metrics import mean_squared_error
from sklearn.model_selection import train_test_split

import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense

def read_data():
    # Load the input file (q_ds.csv) into a pandas DataFrame
    input_df = pd.read_csv('data/q_ds.csv')

    # Load the output file (J_ds.csv) into a pandas DataFrame
    output_df = pd.read_csv('data/J_ds.csv')

    # Convert the DataFrames to arrays
    input_array = input_df.values
    input_array = np.hstack((np.cos(input_array), np.sin(input_array)))
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
        Dense(input_shape=(14,), units=256, activation='tanh'),  
        Dense(128, activation='tanh'), 
        Dense(21, activation='linear'),  
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
    mlp.save('saved_models/neural_jacobian')
    return mlp

def train_MLP(input_train, output_train):
    # Train the multi-layer perceptron using tensorflow
    mlp = Sequential([
        Dense(input_shape=(14,), units=256, activation='tanh'),  
        Dense(128, activation='tanh'), 
        Dense(21, activation='linear'),  
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
    return mlp

def evaluate_MLP(mlp, input_test, output_test):
    # Evaluate the performance of the model on the test set
    # test_score = mlp.score(input_test, output_test)
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

def plot_histograms(input_train, input_test, output_train, output_test):
    # Plot histograms of the input data
    plt.figure(figsize=(14, 6))
    for i in range(input_train.shape[1]):
        plt.subplot(2, input_train.shape[1] // 2, i+1)
        plt.hist(input_train[:,i], bins=50, alpha=0.5, label='Train')
        plt.hist(input_test[:,i], bins=50, alpha=0.5, label='Test')
        if i // 7 < 1:
            plt.xlabel(f'Cosine of joint angle {i+1}')
        else:
            plt.xlabel(f'Sine of joint angle {i-6}')
        plt.legend()
    plt.tight_layout()
    plt.savefig('python_plots/distribution/joint_angle_distribution.png')

    # Plot histograms of the output data
    plt.figure(figsize=(12, 6))
    for i in range(output_train.shape[1]):
        plt.subplot(3, output_train.shape[1]//3, i+1)
        plt.hist(output_train[:,i], bins=50, alpha=0.5, label='Train')
        plt.hist(output_test[:,i], bins=50, alpha=0.5, label='Test')
        plt.xlabel(f'J_{i//7 + 1}{i - 7 * (i//7) + 1}')
        plt.legend()
    plt.tight_layout()
    plt.savefig('python_plots/distribution/jacobian_distribution.png')

def predict(mlp, q):
    return mlp.predict(q)

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
        plot_histograms(input_train, input_test, output_train, output_test)
        print('Histograms plotted.')
        
