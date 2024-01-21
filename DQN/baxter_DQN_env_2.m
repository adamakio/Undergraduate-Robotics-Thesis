%% Environment for DQN implementation
mdl_baxter;

% Observation information
ObservationInfo = rlNumericSpec([7 1]);
ObservationInfo.Name = 'Joint Angles';
ObservationInfo.Description = 'q1, q2, q3, q4, q5, q6, q7';
ObservationInfo.LowerLimit = - pi;
ObservationInfo.UpperLimit = pi;

% Action information
values = [-1 0 1]; % possible values (decrease, keep constant, increase)
combinations = combvec(values, values, values, values, values, values, values); % generate all possible combinations
elements = num2cell(combinations', 2);
ActionInfo = rlFiniteSetSpec(elements);
ActionInfo.Name = 'Joint Actions';
ActionInfo.Description = "Combinations of changes to each joint angle";

% Reset and Step Functions
max_traj_points = 301;
t = linspace(0, 1, max_traj_points);
T = left.fkine(qr);
x0 = T.t;
traj = x0 + t.*[0; 0; -0.3];
ResetHandle = @()myResetFunction2(qr);
StepHandle = @(Action,LoggedSignals) myStepFunction2(Action,LoggedSignals,left, traj);

% Initialize environment
env = rlFunctionEnv(ObservationInfo,ActionInfo,StepHandle,ResetHandle);