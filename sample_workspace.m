clc; clear; close all;

% Load the robot model
mdl_baxter;

% Set the possible q values of the workspace
q_vals = linspace(-pi, pi, 10);

% Generate dataset of joint values
q_ds = combvec(q_vals, q_vals, q_vals, q_vals, q_vals, q_vals, q_vals)';

% Record number of samples
n_samples = size(q_ds, 1);

% Generate dataset of cartesian position values
x_ds = zeros(n_samples, 3);
for i = 1:n_samples
    q = q_ds(i, :);
    T = left.fkine(q); 
    x_ds(i, :) = T.t';
end

% Generate dataset of jacobian matrices
J_ds = zeros(n_samples, 21);
for i = 1:n_samples
    q = q_ds(i, :);
    J = left.jacob0(q); 
    J_reduced = J(1:3, :);
    J_ds(i, :) = reshape(J_reduced, 1, []);
end
