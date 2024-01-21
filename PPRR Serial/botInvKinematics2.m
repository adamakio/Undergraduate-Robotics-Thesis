function [x,y,th1,th2] = botInvKinematics2(xe,ye,r0)
% function [x,y,phi,th1,th2] = botInvKinematics(xe,ye,r0)
%
% Computes inverse kinematics of a mobile robot with 2-Link manipulator.
%
% Inputs:
%   xe - Cartesian x-location of end effector.
%   ye - Cartesian y-location of end effector.
%   r0 - Initial guess for generalized coordinate solution.
%
% Outputs:
%   x   - Cartesian x-location of mobile robot center.
%   y   - Cartesian y-location of mobile robot center.
%   phi - Angle between inerital x-axis and body frame x-axis.
%   th1 - Relative joint angle for link 1.
%   th2 - Relative joint angle for link 2.

% a1 = 4; a2 = 3;
global a1 a2
x = r0(1);
y = r0(2);
th1 = r0(3);
th2 = r0(4);
phi2 = th1;
phi3 = phi2+th2;

epsilon = 0.000001;

f = [x - xe + a1*cos(phi2) + a2*cos(phi3); ...
     y - ye + a1*sin(phi2) + a2*sin(phi3)];
gradf = [1 0 -a1*sin(phi2)-a2*sin(phi3) -a2*sin(phi3); ...
         0 1 a1*cos(phi2)+a2*cos(phi3) a2*cos(phi3)];

flag = 0;
counter = 1;
while flag == 0
    temp = f;
    dx = pinv(gradf)*f ;
    r0 = r0 - dx;
    x = r0(1);
    y = r0(2);
    th1 = r0(3);
    th2 = r0(4);
    phi2 = th1;
    phi3 = phi2+th2;
    f = [x - xe + a1*cos(phi2) + a2*cos(phi3); ...
         y - ye + a1*sin(phi2) + a2*sin(phi3)];
    gradf = [1 0 -a1*sin(phi2)-a2*sin(phi3) -a2*sin(phi3); ...
             0 1 a1*cos(phi2)+a2*cos(phi3) a2*cos(phi3)];
    if (max(abs(dx))<epsilon)&&(max(abs(f))<epsilon)
        break;
    end
    counter = counter + 1;
end

x = r0(1);
y = r0(2);
th1 = r0(3);
while (2*pi<th1)||(0>th1)
    th1 = th1 - sign(th1)*2*pi;
end
th2 = r0(4);
while (2*pi<th2)||(0>th2)
    th2 = th2 - sign(th2)*2*pi;
end