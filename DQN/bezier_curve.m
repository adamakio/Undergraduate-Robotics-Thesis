% x0 = [0.1336, -0.7080, 0.9739];
% [xd, xddot] = bezier_curve_3d(x0, 100);
[xd, xddot] = bezier_curve_3d(x0', N);

function [xd, xddot] = bezier_curve_3d(x0, num_points)
    % Define control points
    P0 = x0;
    P1 = x0 - [0.2, 0, 0];
    P2 = x0 - [0.2, 0, 0.5];
    P3 = x0 - [0.1, 0, 0.5];

    % Calculate the Bezier curve points
    t = linspace(0, 1, num_points);
    B = zeros(num_points, 3);
    dB = zeros(num_points, 3);

    for i = 1:num_points
        B(i, :) = decasteljau([P0; P1; P2; P3], t(i));
        dB(i, :) = bezier_derivative([P0; P1; P2; P3], t(i));
    end
    xd = B';
    xddot = dB';

    % Plot the Bezier curve and its derivative
%     figure;
%     plot3(B(:, 1), B(:, 2), B(:, 3), 'b', 'LineWidth', 1.25);
%     hold on;
end

function Bt = decasteljau(P, t)
    if size(P, 1) == 1
        Bt = P;
    else
        Bt = (1 - t) * decasteljau(P(1:end-1, :), t) + t * decasteljau(P(2:end, :), t);
    end
end

function dBt = bezier_derivative(P, t)
    n = size(P, 1) - 1;
    Q = diff(P, 1, 1);
    dBt = n * decasteljau(Q, t);
end
