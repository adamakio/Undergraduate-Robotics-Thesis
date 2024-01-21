function [InitialObservation, LoggedSignal] = myResetFunction(qr)
% Reset function to place manipulator environment into an initial state.
% Clear figure
clf;

% Return initial environment state variables as logged signals.
LoggedSignal.State = qr';
LoggedSignal.PrevFitness = NaN;
InitialObservation = LoggedSignal.State;

end
