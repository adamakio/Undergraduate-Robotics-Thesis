% Create a predefined environment interface for the system.
env = rlPredefinedEnv("CartPole-Discrete");

% Get the observation and action specification information.
obsInfo = getObservationInfo(env);
actInfo = getActionInfo(env);

% Fix the random generator seed for reproducibility.
rng(0);

% Define the network as an array of layer objects, and get the dimension of 
% the observation space and the number of possible actions from the
% environment specification objects.
net = [
    featureInputLayer(obsInfo.Dimension(1))
    fullyConnectedLayer(20)
    reluLayer
    fullyConnectedLayer(length(actInfo.Elements))
    ];

% Convert to dlnetwork.
net = dlnetwork(net);

% Create the critic approximator using net and the environment specifications. 
critic = rlVectorQValueFunction(net,obsInfo,actInfo);

% Check the critic with a random observation input.
% getValue(critic,{rand(obsInfo.Dimension)})

% Create the DQN agent using critic.
agent = rlDQNAgent(critic);

% Check the agent with a random observation input.
% getAction(agent,{rand(obsInfo.Dimension)})

% Specify the DQN agent options, including training options for the critic.
agent.AgentOptions.UseDoubleDQN = false;
agent.AgentOptions.TargetSmoothFactor = 1;
agent.AgentOptions.TargetUpdateFrequency = 4;
agent.AgentOptions.ExperienceBufferLength = 1e5;
agent.AgentOptions.MiniBatchSize = 256;
agent.AgentOptions.CriticOptimizerOptions.LearnRate = 1e-3;
agent.AgentOptions.CriticOptimizerOptions.GradientThreshold = 1;

% Specify the training options
trainOpts = rlTrainingOptions(...
    MaxEpisodes=1000, ...
    MaxStepsPerEpisode=500, ...
    Verbose=false, ...
    Plots="training-progress",...
    StopTrainingCriteria="AverageReward",...
    StopTrainingValue=480); 

% Visualize environment
plot(env)

% Train the agent using the train function.
doTraining = false;
if doTraining
    % Train the agent.
    trainingStats = train(agent,env,trainOpts);
else
    % Load the pretrained agent for the example.
    load("MATLABCartpoleDQNMulti.mat","agent")
end

% To validate the performance of the trained agent, simulate it within the 
% cart-pole environment. 
simOptions = rlSimulationOptions(MaxSteps=500);
experience = sim(env,agent,simOptions);
totalReward = sum(experience.Reward)