clc;
clear;

lim = [6, 20;
    6, 20;
    0, 5]; 

l     = zeros(1,5);
l(1) = 10; 
l(2) = 10;
l(3) = 12.5;
l(4) = 0;
l(5) = 17.5;

x = 12;
y = 12;
z = 3;

R = [0, 0, 1;
    1, 0, 0;
    0, 1, 0];

p = [x; y; z];

T05 = [R, p;
     0, 0, 0, 1];

q = zeros(1,5);              % joint coordinates from inverse kinematics

% Moving in Y direction of TCP origin by length of l(5) to reach wrist
v1 = [0; 0; 0; 1];
v2 = [0; 1; 0; 0];
wrist = (T05 * v1) + (l(5) * (T05 * v2));

x_wrist = wrist(1);          % cartesian x, y and z coordinates
y_wrist = wrist(2);
z_wrist = wrist(3);

li_xcomp = sqrt(x_wrist^2 + y_wrist^2);
li_ycomp = z_wrist - l(1);
li = sqrt(li_xcomp^2 + li_ycomp^2);

alpha1 = acos( (li^2 + l(3)^2 - l(2)^2) / (2 * li * l(3)) );
beta1 = acos( (li^2 + l(2)^2 - l(3)^2) / (2 * li * l(2)) );
phi1 = atan2((z_wrist - l(1)), li_xcomp);

q(1) = atan2(y_wrist, x_wrist);
q(2) = pi/2 - (phi1 + beta1);
q(3) = -(pi/2 - (alpha1 + beta1));
q(4) = -(q(2) + q(3));
q(5) = q(1);

%
disp("q1 = " + q(1)*180/pi);
disp("q2 = " + q(2)*180/pi);
disp("q3 = " + q(3)*180/pi);
disp("q4 = " + q(4)*180/pi);
disp("q5 = " + q(5)*180/pi);
%}

