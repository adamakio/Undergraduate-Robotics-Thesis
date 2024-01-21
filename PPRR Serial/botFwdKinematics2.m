function [xe,ye] = botFwdKinematics2(x,y,th1,th2)
% function [xe,ye] = botFwdKinematics(x,y,phi,th1,th2)
%
% Computes forward kinematics of a mobile robot with 2-Link manipulator.
%
% Inputs:
%   x   - Cartesian x-location of mobile robot center.
%   y   - Cartesian y-location of mobile robot center.
%   phi - Angle between inerital x-axis and body frame x-axis.
%   th1 - Relative joint angle for link 1.
%   th2 - Relative joint angle for link 2.
%
% Outputs:
%   xe - Cartesian x-location of end effector.
%   ye - Cartesian y-location of end effector.

global a1 a2
% a1 = 4; a2 = 3;
phi2 = th1;
phi3 = phi2+th2;

r = [x + a1*cos(phi2) + a2*cos(phi3); ...
     y + a1*sin(phi2) + a2*sin(phi3)];
xe = r(1);
ye = r(2);