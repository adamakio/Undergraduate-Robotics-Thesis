clc; clear; close all; 

% Import neural kinematics
NK_path = fileparts(which("run_neural_kinematics.py"));
if count(py.sys.path, NK_path) == 0
    insert(py.sys.path, int32(0), NK_path);
end

% Initialize robot
mdl_baxter;

% Define the initial joint angles
q = qr';

% Define the time step
dt = 0.001;
zeta = 0.01; 

% Define the number of iterations
N = 5000;

% Define the desired end-effector trajectory
% Circular trajectory
% t = linspace(0, 4*pi, N);
% xd = [0.5; 0.5; 0.5] + [zeros(size(t)); 0.15*cos(t); 0.15*sin(t)];
% xddot = [zeros(size(t)); -0.15*sin(t); 0.15*cos(t)];
% Square trajectory
square_trajectory;

% Initialize the error
e = zeros(3,N);

% Initialize costate
w = zeros(3,1);

% Initialize history
qdot_history = zeros(7, N);
q_history = zeros(N, 7);
J_tilde_history = zeros(N, 1);

% Load weights needed for jacobian
weights = py.run_neural_kinematics.get_weights();
w1 = weights(1); w1 = w1{1};
b1 = weights(2); b1 = b1{1};
w2 = weights(3); w2 = w2{1};
b2 = weights(4); b2 = b2{1};
w3 = weights(5); w3 = w3{1};
b3 = weights(6); b3 = b3{1};

% Implement the control loop
for i = 1:N
    % Compute the forward kinematics
    T = left.fkine(q');
    x = T.t;

    % Compute the error between the desired and actual end-effector position and orientation
    e(1:3, i) = x - xd(:, i);
    
    % Compute the Jacobian matrix using neural jacobian
    J = py.run_neural_kinematics.jacobian_autograd(q', w1, b1, w2, b2, w3, b3);
    J = double(J);
    J_exact = left.jacob0(q);
    J_exact = J_exact(1:3, :); % only DoF of interest
    
    J_tilde_history(i) = norm(J-J_exact, "fro")/norm(J_exact, "fro");
    
    % Compute the control inputs (joint velocities)
    qdot = - J' * e(:, i) / dt + J' * w;

    % Projection operation
    % Convex S = [-2, 2]^n
    qdot = min(2*ones("like", qdot),qdot);
    qdot = max(-2*ones("like", qdot),qdot);
    % Non convex S = {x: x=+/-1 or |x|<0.1}
    % qdot = projection(qdot)';

    % Update the joint angles
    q = q + qdot * dt;
    q_history(i, :) = q';
    qdot_history(:, i) = qdot;

    % Update the costate
    wdot = (xddot(:, i) - J * J' * w) / zeta; % Will always be zero
    w = w + wdot * dt;
    
end

% Plot the robot
left.plot(q_history, 'nowrist', 'noname', 'notiles', ...
    'nojoints', 'delay', 1e-7, 'trail', {'r', 'LineWidth', 2}, 'ortho', 'movie', 'tvr_neural_kinematics.mp4');
    
% Update the plot
drawnow;

figure(2)
plot(t, e(1,:)); hold on;
plot(t, e(2,:));
plot(t, e(3,:));
legend('e_{x}', 'e_{y}', 'e_{z}')
xlabel('Time (s)')
ylabel('Error (m)')

figure(3)
plot(t, qdot_history(1,:)); hold on;
plot(t, qdot_history(2,:));
plot(t, qdot_history(3,:));
plot(t, qdot_history(4,:));
plot(t, qdot_history(5,:));
plot(t, qdot_history(6,:));
plot(t, qdot_history(7,:));
legend('u_{1}', 'u_{2}', 'u_{3}', 'u_{4}', 'u_{5}', 'u_{6}', 'u_{7}')
xlabel('Time (s)')
ylabel('Control input (rad/s)')

figure(4)
plot(t, J_tilde_history);
ymin = min(J_tilde_history(:)) - 1e-3;
ymax = max(J_tilde_history(:)) + 1e-3;
ylim([ymin ymax])
xlabel('Time (s)')
ylabel('$\|\bf{\tilde{J}}\|_F / \|\bf{J}\|_F$', 'Interpreter', 'latex')
