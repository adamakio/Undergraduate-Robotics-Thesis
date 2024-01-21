clc; clear; close all; 

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
t = linspace(0, 4*pi, N);
xd = [0.5; 0.5; 0.5] + [0.15*cos(t); 0.15*sin(t); zeros(size(t))];
xddot = [-0.15*sin(t); 0.15*cos(t); zeros(size(t))];
% Square trajectory
% square_trajectory;

% Plot the 3D Square Trajectory and its Derivative
% figure;
% subplot(2, 1, 1);
% plot3(xd(:, 1), xd(:, 2), xd(:, 3), 'LineWidth', 2);
% xlabel('X (m)');
% ylabel('Y (m)');
% zlabel('Z (m)');
% title('Square Trajectory in 3D');
% grid on;

% Initialize the error
e = zeros(3,N);

% Initialize costate
w = zeros(3,1);

% Initialize history
qdot_history = zeros(7, N);
q_history = zeros(N, 7);
J_history = zeros(N, 21);

% Implement the control loop
for i = 1:N
    % Compute the forward kinematics
    T = left.fkine(q');
    x = T.t;
    % R = zeros(3,3);
    % R(:,1) = T.n; R(:,2) = T.o; R(:,3) = T.a;

    % Compute the error between the desired and actual end-effector position and orientation
    e(1:3, i) = x - xd(:, i);
    % e(4:6, i) = tr2rpy(R) - tr2rpy(Rd);
    
    % Compute the Jacobian matrix
    J = left.jacob0(q);
    J = J(1:3, :); % only DoF of interest
    J_history(i, :) = reshape(J, 1, []);
    
    % Compute the control inputs (joint velocities)
    qdot = - J' * e(:, i) / dt + J' * w;

    % Projection operation
    % Convex S = [-2, 2]^n
    % qdot = min(2*ones("like", qdot),qdot);
    % qdot = max(-2*ones("like", qdot),qdot);
    % Non convex S = {x: x=+/-1 or |x|<0.1}
    qdot = projection(qdot)';

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
    'nojoints', 'delay', 0, 'trail', {'r', 'LineWidth', 2});%, 'ortho', 'movie', 'tvr_nonconvex.mp4');

% Update the plot
drawnow;

figure(2)
plot(t, e(1,:), 'LineWidth', 1.5); hold on; grid on;
plot(t, e(2,:), 'LineWidth', 1.5);
plot(t, e(3,:), 'LineWidth', 1.5);
legend('e_{x}', 'e_{y}', 'e_{z}', 'interpreter','latex', location='best')
xlabel('Time (s)','interpreter','latex')
ylabel('Error (m)','interpreter','latex')


figure(3)
plot(t, qdot_history(1,:), 'LineWidth', 1.5); hold on; grid on;
plot(t, qdot_history(2,:), 'LineWidth', 1.5);
plot(t, qdot_history(3,:), 'LineWidth', 1.5);
plot(t, qdot_history(4,:), 'LineWidth', 1.5);
plot(t, qdot_history(5,:), 'LineWidth', 1.5);
plot(t, qdot_history(6,:), 'LineWidth', 1.5);
plot(t, qdot_history(7,:), 'LineWidth', 1.5);
legend('u_{1}', 'u_{2}', 'u_{3}', 'u_{4}', 'u_{5}', 'u_{6}', 'u_{7}', 'interpreter', 'latex', Location='best')
xlabel('Time (s)','interpreter','latex')
ylabel('Control input (rad/s)','interpreter','latex')

figure(4)
plot(t, q_history(:, 1), 'LineWidth', 1.5); hold on; grid on;
plot(t, q_history(:, 2), 'LineWidth', 1.5);
plot(t, q_history(:, 3), 'LineWidth', 1.5);
plot(t, q_history(:, 4), 'LineWidth', 1.5);
plot(t, q_history(:, 5), 'LineWidth', 1.5);
plot(t, q_history(:, 6), 'LineWidth', 1.5);
plot(t, q_history(:, 7), 'LineWidth', 1.5);
legend('\theta_{1}', '\theta_{2}', '\theta_{3}', '\theta_{4}', '\theta_{5}', '\theta_{6}', '\theta_{7}','interpreter','latex', Location='best')
xlabel('Time (s)','interpreter','latex')
ylabel('Control input (rad/s)','interpreter','latex')