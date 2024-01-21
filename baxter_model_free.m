clc; clear; close all; 

% Initialize robot
mdl_baxter;

% Define the initial joint angles
q = qr';

% Define the number of iterations
N = 10000;

% Define the desired end-effector trajectory
% Circular trajectory
t = linspace(0, 4*pi, N);
xd = [0.5; 0.5; 0.5] + [zeros(size(t)); 0.15*cos(t); 0.15*sin(t)];
xddot = [zeros(size(t)); -0.15*sin(t); 0.15*cos(t)];
% Square trajectory
% square_trajectory;

% Define the scaling factors
dt = t(2) - t(1);
epsilon = 0.01;
zeta = 0.01;
nu = 0.001;

% Initialize the error
e = zeros(3,N);

% Initialize costate and jacobian
w = zeros(3,1);
J_hat = randn(3, 7);

% Initialize history
qdot_history = zeros(7, N);
q_history = zeros(N, 7);
J_tilde_history = zeros(N, 1);
% J_hat_history = zeros(N, 1);

% Upper bound on noise
rho_0 = 0.002;

% Set tolerances for costate and Jacobian estimate
tol_J = 1e-10;
tol_w = 1e-10;

% Implement the control loop
for i = 1:N
    % Compute the forward kinematics
    if i == 1
        T = left.fkine(q');
        x = T.t;
    else
        x = new_x;
    end

    % Compute the error between the desired and actual end-effector position and orientation
    e(1:3, i) = x - xd(:, i);
    
    % Compute the control inputs (joint velocities)
    qdot_bar = - J_hat' * e(:, i) / epsilon + J_hat' * w;

    % Projection operation
    % Convex S = [-2, 2]^n
    % qdot_bar = min(2*ones(7, 1), qdot_bar);
    % qdot_bar = max(-2*ones(7, 1), qdot_bar);
    % Non convex S = {x: x=+/-1 or |x|<0.1}
    qdot_bar = projection(qdot_bar)';

    % Add noise
    rho = normrnd(0, 0.001, 7, 1);
    rho = min(rho_0*ones(7, 1), rho);
    rho = max(-rho_0*ones(7, 1), rho);
    qdot = qdot_bar + rho; 

    % Update the joint angles
    q = q + qdot * dt;
    qdot_history(:, i) = qdot;
    q_history(i, :) = q';

    % Compute the Jacobian matrix
    J = left.jacob0(q);
    J = J(1:3, :); % only DoF of interest

    % Measure velocity in cartesian space
    T = left.fkine(q');
    new_x = T.t;
    xdot = (new_x - x) / dt;

    % Update the jacobian
    J_hat_dot = -(J_hat*qdot - xdot)*qdot';
    while any(any(J_hat_dot > tol_J))
        J_hat = J_hat + J_hat_dot * dt;
        J_hat_dot = -(J_hat*qdot - xdot)*qdot';
    end
    J_tilde_history(i) = norm(J-J_hat, "fro") / norm(J, 'fro');
    % J_hat_history(i, :) = reshape(J_hat, 1, []);

    % Update the costates
    wdot = (xddot(:, i) - J_hat * J_hat' * w) / zeta;
    if isnan(max(wdot))
        disp(i);
        return;
    end
    w = w + wdot * dt;


end

% Plot the robot
left.plot(q_history, 'nowrist', 'noname', 'notiles', ...
    'nojoints', 'delay', 0, 'trail', {'r', 'LineWidth', 2});%, 'ortho', 'movie', 'tvr_mf_nonconvex.mp4');
    
% Update the plot
drawnow;

figure(2)
plot(t, e(1,:)); hold on; grid on;
plot(t, e(2,:));
plot(t, e(3,:));
legend('e_{x}', 'e_{y}', 'e_{z}')
xlabel('Time (s)')
ylabel('Error (m)')

figure(3)
plot(t, qdot_history(1,:)); hold on; grid on;
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
ymin = min(J_tilde_history) - 1e-3;
ymax = max(J_tilde_history) + 1e-3;
ylim([ymin ymax])
xlabel('Time (s)')
ylabel('$\frac{\|\bf{\tilde{J}}\|_F}{\|\bf{J}\|_F}$', 'Interpreter', 'latex')

