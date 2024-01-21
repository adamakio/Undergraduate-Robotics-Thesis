function plotbot2(x,y,theta1,theta2)
global a1 a2
% function plotbot(x,y,phi,theta1,theta2)
%
% Plots the locatoin of a mobile robot with 2-Link manipulator.
%
% Inputs:
%   x   - Cartesian x-location of mobile robot center.
%   y   - Cartesian y-location of mobile robot center.
%   phi - Angle between inerital x-axis and body frame x-axis.
%   th1 - Relative joint angle for link 1.
%   th2 - Relative joint angle for link 2.

% Initialize physical dimension of robot
% a1 = 4; a2 = 3;

phi2 = theta1;
phi3 = theta1+theta2;
st   = 0.1;

% Initialize Rotation Matrices
R2 = [cos(phi2) -sin(phi2); sin(phi2) cos(phi2)];
R3 = [cos(phi3) -sin(phi3); sin(phi3) cos(phi3)];

j1 = [x; y];
j2 = j1 + [cos(phi2); sin(phi2)]*a1;
j3 = j2 + [cos(phi3); sin(phi3)]*a2;

L1 = [-2*st -2*st a1+st a1+st; st -st -st st];
L2 = [-st -st a2 a2; st -st -st st];
EE = [0.5 0 0 0.5; -0.25 -0.25 0.25 0.25];

for i = 1:4,
    L1(:,i) = R2*L1(:,i) + j1;
    L2(:,i) = R3*L2(:,i) + j2;
    EE(:,i) = R3*EE(:,i) + j3;
end
sp2 = 400;
L1(:,5) = L1(:,1);
L2(:,5) = L2(:,1);
L12 = [linspace(L1(1,1),L1(1,2),sp2); ...
       linspace(L1(2,1),L1(2,2),sp2)];
L13 = [linspace(L1(1,4),L1(1,3),sp2); ...
       linspace(L1(2,4),L1(2,3),sp2)];
L14 = zeros(2,2*sp2);
for k = 1:sp2,
    L14(:,2*k-1) = L12(:,k);
    L14(:,2*k) = L13(:,k);
end
L22 = [linspace(L2(1,1),L2(1,2),sp2); ...
       linspace(L2(2,1),L2(2,2),sp2)];
L23 = [linspace(L2(1,4),L2(1,3),sp2); ...
       linspace(L2(2,4),L2(2,3),sp2)];
L24 = zeros(2,2*sp2);
for k = 1:sp2,
    L24(:,2*k-1) = L22(:,k);
    L24(:,2*k) = L23(:,k);
end
base_rad=0.5;
nop=base_rad*200;
base_lsp=linspace(0,2*pi,nop);
rho=ones(1,nop)*0.5;
[basex,basey]=pol2cart(base_lsp,rho);
basex=basex+x;
basey=basey+y;
% Plotting code
hold on;
plot(EE(1,:),EE(2,:),'k');
plot(L1(1,:),L1(2,:),'b',L2(1,:),L2(2,:),'g');
plot(L14(1,:),L14(2,:),'b',L24(1,:),L24(2,:),'g');
fill(basex,basey,'r');
plot(basex,basey,'r');
axis ([-15 15 -15 15]);