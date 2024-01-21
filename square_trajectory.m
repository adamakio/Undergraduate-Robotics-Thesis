% Square trajectory
% Square Trajectory Parameters
edge_length = 0.2; % meters
n_points_per_edge = 500;
total_points = 4 * n_points_per_edge;
Z = 0.5;

% Initialize the trajectory and its derivative
xd = zeros(total_points, 3);
xddot = zeros(total_points, 3);

% Calculate the time step for each point assuming constant velocity along the trajectory
time_step = 0.01; % seconds

% Generate the trajectory and its derivative
for i = 1:total_points
    if i <= n_points_per_edge
        t = (i - 1) / n_points_per_edge;
        xd(i, :) = [t * edge_length, 0, Z];
    elseif i <= 2 * n_points_per_edge
        t = (i - n_points_per_edge - 1) / n_points_per_edge;
        xd(i, :) = [edge_length, t * edge_length, Z];
    elseif i <= 3 * n_points_per_edge
        t = (i - 2 * n_points_per_edge - 1) / n_points_per_edge;
        xd(i, :) = [(1 - t) * edge_length, edge_length, Z];
    else
        t = (i - 3 * n_points_per_edge - 1) / n_points_per_edge;
        xd(i, :) = [0, (1 - t) * edge_length, Z];
    end
    
    % Compute the derivative
    if i == 1 || i == n_points_per_edge + 1 || i == 2 * n_points_per_edge + 1 || i == 3 * n_points_per_edge + 1
        xddot(i, :) = [0, 0, 0];
    elseif i > 1
        xddot(i, :) = (xd(i, :) - xd(i - 1, :)) / time_step;
    end
end

xd = xd' + [0.4; 0.4; 0];
N = 2*total_points;
t = linspace(0, time_step*N, N);
xd = [xd xd];
xddot = [xddot' xddot'];