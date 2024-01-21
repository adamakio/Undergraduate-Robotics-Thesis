clc; clear; close all; 

% Initialize robot
mdl_baxter;

% Define the initial joint angles
q = qr';

% Set hyperparameters
max_traj_points = 100; % Maximum number of trajectory points
max_episodes = 1000; % Maximum number of episodes per trajectory point
batch_size = 64; % Size of minibatch used for updating the critic network
gamma = 0.9; % Discount factor for future rewards
exploration_factor = 1; % Initial exploration factor
min_exploration_factor = 0.01; % Minimum exploration factor
exploration_decay_rate = 0.001; % Exploration decay rate
learning_rate = 0.01; % Learning rate for updating the critic network
goal_fitness = 0.001; % Goal fitness for reward calculation and early stop

% Initialize trajectory using Bezier curve
t = linspace(0, 1, max_traj_points);
T = left.fkine(q');
x0 = T.t;
traj = x0 + t.*[0; 0; -0.2];

% Initialize initial joint positions
theta = zeros(7, max_traj_points); 
theta(:, 1) = q;

% Initialize experience buffer
exp_buffer = createExperienceBuffer();

% Loop through trajectory points
for j = 2:max_traj_points
    
    % Loop through episodes
    for i = 1:max_episodes
        
        % Observe states from previous trajectory point
        states = theta(:, j-1);
        
        % Select action based on DQN Critic network's weights and exploration factor
        if rand < explorationFactor
            % Explore by selecting a random action
            action = randi(numActions);
        else
            % Exploit by selecting the action with the highest Q-value
            Qvalues = predict(criticNet, state);
            [~, action] = max(Qvalues);
        end
        
        % Update joint positions by amount delta_theta according to selected action
        if action == 1
            delta_theta = 4.3633e-05; 
        elseif action == 2
            delta_theta = - 4.3633e-05; 
        else
            delta_theta = 0;
        end
        q = theta(:,j-1) + delta_theta;
        theta(:,j) = q;
        
        % Perform action
        x = left.fkine(q').t;
        
        % Evaluate fitness
        if i > 1
            prev_fitness = fitness;
        end
        fitness = norm(x - traj(:, j)); 
        

        % Obtain reward
        if i == 1
            reward = 0;
        else
            reward = 100 * arctan(0.5*pi*(prev_fitness - fitness)/goal_fitness);
        end
        
        % Store the experience in the ExperienceBuffer
        experience = struct('state', state, 'action', action, 'reward', R, 'nextState', q', 'done', false);
        exp_buffer = addExperience(exp_buffer, experience);
    
        % Sample a minibatch of transitions from the ExperienceBuffer
        minibatch = sampleExperienceBuffer(exp_buffer, batch_size);
    
        % Calculate the DQN Critic network loss and update the network weights
        loss = calculateCriticLoss(criticNet, minibatch, gamma, learningrate);
        criticNet = updateCriticNetwork(criticNet, loss);
    
        % Update the exploration factor
        exploration_factor = max(min_exploration_factor, exploration_factor - exploration_decay_rate);

        % Check for goal fitness
        if fitness <= goal_fitness
            break;
        end
    end
end