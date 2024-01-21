function result = botThetaDot2(r,reDot,P,Q)
% function result = botThetaDot(r,reDot,P,Q)
%
% Computes time rate of change of generalized coordiantes based on weighted
% pseudoinverse solution.  Constraint for rolling without slipping
% condition is included in the Jacobian.
%
% Inputs:
%   r     - current values for general coordinates.
%   reDot - desired end effector velocity.
%   P     - weight matrix for pseudoinverse solution.
%   Q     - weight matrix for pseudoinverse solution.
%
% Outputs:
%   result - time rate of change of generalized coordiantes.
global a1 a2
%x = r(1); y = r(2); 
th1 = r(3); th2 = r(4);
% a1 = 4; a2 = 3;
phi2 = th1;
phi3 = phi2+th2;
J = [1 0 -a1*sin(phi2)-a2*sin(phi3) -a2*sin(phi3); ...
     0 1 a1*cos(phi2)+a2*cos(phi3) a2*cos(phi3);];
qBar = sqrt(Q);
pBar = sqrt(P);
result = inv(qBar)*pinv(pBar*J*inv(qBar))*pBar*reDot;