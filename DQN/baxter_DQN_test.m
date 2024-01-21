% Number of points
N = 300;

% Angle increment
dq = deg2rad(0.5); 

q_history = zeros(7, N);
q_history(:, 1) = qr';

for i = 1:N-1
    q = q_history(:, i);
    % [~, idx] = max(getValue(critic,{q}));
    % action = combinations(:, idx);
    action = cell2mat(getAction(agent,{q}));
    q_history(:, i+1) = q + dq * action';
end
clf;
% Plot the robot
left.plot(q_history', 'nowrist', 'noname', 'notiles', ...
    'nojoints', 'delay', 0, 'trail', {'r', 'LineWidth', 2});