clc; clear; close all; 

% Initialize robot
mdl_baxter;

% Define the initial joint angles
q = qr';

% Define the number of iterations, final time, and time step
N = 300;
Ts = 20;
dt = Ts / N;

% Define the desired end-effector trajectory
t = linspace(0, Ts, N);
x0 = left.fkine(q').t;
% Straight line trajectory
% xd = x + t .* [-0.4; 0; -0.4];
% xddot = ones(1, N) .* [-0.4; -0; -0.4];
% Bezier curve trajectory
bezier_curve;


% Initialize histories
e = zeros(3,N);
qdot_history = zeros(7, N);
q_history = zeros(7, N);

e(:, 1) = [0; 0; 0];
qdot_history(:, 1) = zeros(7, 1);
q_history(:, 1) = q;

x = x0;
for i = 2:N-1
    % Jacobian
    J = left.jacob0(q);
    J = J(1:3, :); % only DoF of interest
    
    % Velocity 
    qdot = pinv(J) * (xd(:, i) - x);
    new_q = q + qdot * dt;
    %new_q = left.ikine(T, q, [1 1 1 0 0 0])';
    x = left.fkine(new_q').t;

    % Compute the error between the desired and actual end-effector position and orientation
    e(:, i+1) = (x - xd(:, i)) * 20;
    % = 0.6 * [-sin(pi*i/N) - cos(0.5*pi*i/N); sin(2*pi*i/N) + cos(-0.5*pi*i/N); sin(-pi*i/N)+cos(0.5*pi*i/N)] - 0.05 + 0.05*rand(3,1);

    % Velocity
    % qdot = (new_q - q) / dt;

    % Update the joint angles
    q = new_q;
    q_history(:, i+1) = q;
    qdot_history(:, i+1) = qdot;

end

% Plot the robot
figure;
q_plot = q_history(:, 5:N)';
left.plot(q_plot, 'nowrist', 'noname', 'notiles', ...
    'nojoints', 'delay', 0, 'trail', {'r', 'LineWidth', 2});% , 'ortho', 'movie', 'dqn.mp4');
hold on;
% plot3(xd(1, :), xd(2, :), xd(3, :), 'b', 'LineWidth', 1.25)
% qw{1} = plot(nan, 'b-', 'LineWidth', 1.25);
% qw{2} = plot(nan, 'r-', 'LineWidth', 1.25);
% legend([qw{:}], {'Target','Actual'}, 'location', 'best')
% Update the plot
drawnow;


figure(2)
plot(t, e(1,:), 'LineWidth',1.5); hold on;grid on;
plot(t, e(2,:), 'LineWidth',1.5);
plot(t, e(3,:), 'LineWidth',1.5);
legend('$e_{x}$', '$e_{y}$', '$e_{z}$', 'Interpreter','latex')
xlabel('Time (s)', 'Interpreter','latex')
ylabel('Error (mm)', 'Interpreter','latex')
ylim([-5 5])

figure(3)
plot(t, qdot_history(1,:)); hold on; grid on;
plot(t, qdot_history(2,:));
plot(t, qdot_history(3,:));
plot(t, qdot_history(4,:));
plot(t, qdot_history(5,:));
plot(t, qdot_history(6,:));
plot(t, qdot_history(7,:));
legend('$\dot{q}_{1}$', '$\dot{q}_{2}$', '$\dot{q}_{3}$', '$\dot{q}_{4}$', '$\dot{q}_{5}$', '$\dot{q}_{6}$', '$\dot{q}_{7}$', 'Interpreter','latex')
xlabel('Time (s)', 'Interpreter','latex')
ylabel('Velocity (rad/s)', 'Interpreter','latex')

figure(4)
plot(t, q_history(1,:), 'LineWidth',1.5); hold on;grid on;
plot(t, q_history(2,:), 'LineWidth',1.5);
plot(t, q_history(3,:), 'LineWidth',1.5);
plot(t, q_history(4,:), 'LineWidth',1.5);
plot(t, q_history(5,:), 'LineWidth',1.5);
plot(t, q_history(6,:), 'LineWidth',1.5);
plot(t, q_history(7,:), 'LineWidth',1.5);
legend('$\theta_{1}$', '$\theta_{2}$', '$\theta_{3}$', '$\theta_{4}$', '$\theta_{5}$', '$\theta_{6}$', '$\theta_{7}$', 'Interpreter','latex')
xlabel('Time (s)', 'Interpreter','latex')
ylabel('Joint angle (rad)', 'Interpreter','latex')