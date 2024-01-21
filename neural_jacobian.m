% Reset windows
clear all; close all; clc;

% Load the input and output data
input_data = table2array(readtable('data/q_history.csv'));
output_data = table2array(readtable('data/J_history.csv'));

% Split the data into a training set and a test set
rng(0); % Set the random number seed for reproducibility
indices = randperm(size(input_data, 1)); % Randomly permute the indices
num_train = floor(0.8 * size(input_data, 1)); % 80% for training
train_indices = indices(1:num_train);
test_indices = indices(num_train+1:end);
input_train = input_data(train_indices,:);
output_train = output_data(train_indices,:);
input_test = input_data(test_indices,:);
output_test = output_data(test_indices,:);

% Train a neural network using the training data
hidden_layer_sizes = [256,128]; % Two hidden layers 
net = fitnet(hidden_layer_sizes);
net.trainFcn = 'trainlm'; % Use Levenberg-Marquardt backpropagation
net.divideParam.trainRatio = 0.8; % 80% of data for training
net.divideParam.valRatio = 0.2; % 20% of data for validation
net.divideParam.testRatio = 0; % 0% of data for testing
[net,tr] = train(net,input_train',output_train');

% Evaluate the neural network on the test data
output_pred = net(input_test')';
mse = mean((output_test - output_pred).^2, 1);
fprintf('Mean squared error: %f\n', mean(mse));

% Plot each component of the predicted array
num_plots = size(output_pred, 2);
for i = 1:num_plots
    subplot(3,7,i);
    plot(output_test(:,i), 'r');
    hold on;
    plot(output_pred(:,i), 'b');
    title(sprintf('Output %d', i));
    xlabel('Sample');
    row = 1 + mod(i-1, 7); col = 1 + rem(i-1, 7);
    ylabel(sprintf('Value of Jacobian at index (%d, %d)', row, col));
    legend('Actual Jacobian', 'Predicted Jacobian');
end
