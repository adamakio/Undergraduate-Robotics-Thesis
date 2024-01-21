function [NextObs,Reward,IsDone,LoggedSignals] = myStepFunction(Action,LoggedSignals, robot, xd)
% Custom step function to construct manipulator environment 
%
% This function applies the given action to the environment and evaluates
% the system dynamics for one simulation step.

% Inputs:
% robot := robot model
% xd    := Desired cartesian position

% Define the environment constants.
delta_theta = deg2rad(0.5); % Increment in joint angle
goal_fitness = 0.05; % Goal fitness for reward calculation and early stop
m = 10; % Scaling factor for reward

% Unpack the state vector from the logged signals.
State = LoggedSignals.State;
PrevFitness = LoggedSignals.PrevFitness;
Index = LoggedSignals.TrajIndex;

% Apply joint change
NextObs = State + delta_theta * Action';
% disp(NextObs);

% Perform action
x = robot.fkine(NextObs').t;
robot.plot(NextObs', 'nowrist', 'noname', 'notiles', 'nojoints'); 
hold on; plot3(x(1), x(2), x(3), '.r', 'MarkerSize', 3); 
if isnan(PrevFitness)
    plot3(0.5, 0.5, 0.5, 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'blue', 'MarkerEdgeColor', 'blue')
end

drawnow;


% Evaluate fitness
Fitness = norm(x - xd); 

% Obtain reward
if isnan(PrevFitness)
    Reward = 0;
else
    Reward = m * atan(0.5*pi*(PrevFitness - Fitness)/goal_fitness);
end

% Check terminal condition.
IsDone = Fitness <= goal_fitness;

% Transform state to observation.
LoggedSignals.State = NextObs;
LoggedSignals.PrevFitness = Fitness;
LoggedSignals.TrajIndex = Index + 1; 

end