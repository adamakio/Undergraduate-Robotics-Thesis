function [InitialObservation, LoggedSignal] = myResetFunction2(qr)
% Reset function to place manipulator environment into an initial state.
% Clear figure
clf;

% Return initial environment state variables as logged signals.
LoggedSignal.State = qr';
LoggedSignal.PrevFitness = NaN;
LoggedSignal.TrajIndex = 1;
InitialObservation = LoggedSignal.State;

end
