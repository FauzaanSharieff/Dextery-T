clc;
clear;

%% DH Parameter Initialization
% Units:
% phi and alpha: radians
% d and a: cm

theta = zeros(1,5); % joint coordinates as an input for Forward kinematics
l     = zeros(1,5); % Constant length (an approximate of the actual length)

phi   = zeros(1,5);
d     = zeros(1,5);
a     = zeros(1,5);
alpha = zeros(1,5);


%% DH parameters 

l(1) = 10; 
l(2) = 10;
l(3) = 12.5;
l(4) = 0;
l(5) = 17.5; 

theta(1) = 0;
theta(2) = 0;
theta(3) = 0;
theta(4) = (pi - (theta(2) + theta(3) + pi/2) - pi/2); 
theta(5) = theta(1);

phi(1)   = theta(1);
d(1)     = l(1);
a(1)     = 0;
alpha(1) = -pi/2;

phi(2)   = theta(2) - pi/2;
d(2)     = 0;
a(2)     = l(2);
alpha(2) = 0;

phi(3)   = theta(3) + pi/2; 
d(3)     = 0;
a(3)     = l(3);
alpha(3) = 0;

phi(4)   = theta(4) + pi; 
d(4)     = 0;
a(4)     = 0;
alpha(4) = pi/2;

phi(5)   = theta(5) + pi/2;
d(5)     = l(5);
a(5)     = 0;
alpha(5) = -pi/2;


%% Calculate Transformation Matrix

T = cell(1,5);

for i = 1:5
    T{i} = [
        cos(phi(i)), -cos(alpha(i))*sin(phi(i)),  sin(alpha(i))*sin(phi(i)), a(i)*cos(phi(i));
        sin(phi(i)),  cos(alpha(i))*cos(phi(i)), -sin(alpha(i))*cos(phi(i)), a(i)*sin(phi(i));
        0,            sin(alpha(i)),              cos(alpha(i)),             d(i);
        0,            0,                          0,                         1
    ];
end

T01 = T{1};
T12 = T{2};
T23 = T{3};
T34 = T{4};
T45 = T{5};

T05 = T01 * T12 * T23 * T34 * T45;

disp("Final Transformation Matrix T05 = ");
disp(T05);


%% Inverse Kinematics 

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
phi1 = atan2(li_ycomp, li_xcomp);
gamma1 = alpha1 + beta1;

q(1) = atan2(y_wrist, x_wrist);
q(2) = pi/2 - (phi1 + beta1);
q(3) = -(pi/2 - (alpha1 + beta1));
q(4) = (pi - (q(2) + q(3) + pi/2) - pi/2);
q(5) = q(1);

%
disp("q1: Set = " + theta(1)*180/pi + ", Calculated = " + q(1)*180/pi);
disp("q2: Set = " + theta(2)*180/pi + ", Calculated = " + q(2)*180/pi);
disp("q3: Set = " + theta(3)*180/pi + ", Calculated = " + q(3)*180/pi);
disp("q4: Set = " + theta(4)*180/pi + ", Calculated = " + q(4)*180/pi);
disp("q5: Set = " + theta(5)*180/pi + ", Calculated = " + q(5)*180/pi);
%}