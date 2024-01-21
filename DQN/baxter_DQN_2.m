clc; clear; close all;

baxter_DQN_env_2;

% obtain observation and action specifications
obsInfo = getObservationInfo(env);
actInfo = getActionInfo(env);

% Deep neural network
dnn = [
    featureInputLayer(prod(obsInfo.Dimension))
    fullyConnectedLayer(128)
    reluLayer()
    fullyConnectedLayer(256)
    reluLayer()
    fullyConnectedLayer(numel(actInfo.Elements))];
dnn = dlnetwork(dnn);

% Create critic
critic = rlVectorQValueFunction(dnn,obsInfo,actInfo);

% Create DQN agent and specify options
agent = rlDQNAgent(critic);

agent.AgentOptions.MiniBatchSize=64;
agent.AgentOptions.UseDoubleDQN=false;
agent.AgentOptions.DiscountFactor=0.9;
agent.AgentOptions.TargetSmoothFactor=1e-3;
agent.AgentOptions.ExperienceBufferLength=10000;
agent.AgentOptions.CriticOptimizerOptions.LearnRate=1e-2;
agent.AgentOptions.CriticOptimizerOptions.GradientThreshold=1;
agent.AgentOptions.EpsilonGreedyExploration.Epsilon = 1;
agent.AgentOptions.EpsilonGreedyExploration.EpsilonMin = 0.01;
agent.AgentOptions.EpsilonGreedyExploration.EpsilonDecay = 0.005;

% Training options 
trainOpts = rlTrainingOptions;
trainOpts.MaxStepsPerEpisode = 300;
trainOpts.MaxEpisodes = 100;
trainOpts.StopTrainingCriteria = "AverageReward";
trainOpts.StopTrainingValue = 2000;
trainOpts.ScoreAveragingWindowLength = 30;
trainOpts.SaveAgentCriteria = 'EpisodeReward';
trainOpts.SaveAgentValue = 1500;
trainOpts.SaveAgentDirectory = pwd + "\run2\Agents";

% Train
doTraining = true;

if doTraining
    % Train the agent.
    trainingStats = train(agent,env,trainOpts);
else
    % Load pretrained agent for the example.
    load('run1\Agents\genericMDPQAgent.mat','agent'); 
end