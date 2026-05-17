clc;
clear;
close all;

%% Dextery-T Workspace: Top-Front Half Torus
% Coordinate convention:
% x = front direction
% y = side direction
% z = vertical direction
%
% Torus parameters:
% R = distance from origin to center of torus tube
% r = radius of torus tube / cross-section

R = 14.5;   % cm
r = 7.5;    % cm

%% Generate Half-Torus Surface
% Parametric torus:
% x = (R + r*cos(u))*cos(v)
% y = (R + r*cos(u))*sin(v)
% z = r*sin(u)
%
% Top half:   u from 0 to pi       gives z >= 0
% Front half: v from -pi/2 to pi/2 gives x >= 0

u = linspace(0, pi, 100);
v = linspace(-pi/2, pi/2, 100);

[U, V] = meshgrid(u, v);

X = (R + r*cos(U)) .* cos(V);
Y = (R + r*cos(U)) .* sin(V);
Z = r*sin(U);

%% Plot Workspace

figure;
surf(X, Y, Z, ...
    'FaceAlpha', 0.35, ...
    'EdgeColor', 'none');

hold on;
grid on;
axis equal;

xlabel('x / front direction [cm]');
ylabel('y / side direction [cm]');
zlabel('z / vertical direction [cm]');
title('Dextery-T Approximate Cartesian Workspace');

view(45, 25);
rotate3d on;

% Add coordinate axes
plot3([0, R+r+3], [0, 0], [0, 0], 'r', 'LineWidth', 2); % x-axis
plot3([0, 0], [-(R+r+3), R+r+3], [0, 0], 'g', 'LineWidth', 2); % y-axis
plot3([0, 0], [0, 0], [0, r+3], 'b', 'LineWidth', 2); % z-axis

text(R+r+3, 0, 0, ' +x/front');
text(0, R+r+3, 0, ' +y/side');
text(0, 0, r+3, ' +z/up');

%% ============================================================
%  SECTION 1: Find x-limits when y and z are known
%  Uncomment this section if you want x-limits.
%% ============================================================

%
givenY = -5;     % cm
givenZ = 3;     % cm

[xMin, xMax, validX] = getXLimits(givenY, givenZ, R, r);

if validX
    fprintf('\nX-limit calculation:\n');
    fprintf('Given y = %.2f cm, z = %.2f cm\n', givenY, givenZ);
    fprintf('xMin = %.2f cm\n', xMin);
    fprintf('xMax = %.2f cm\n', xMax);

    % Plot x-limit points
    plot3(xMin, givenY, givenZ, 'ko', ...
        'MarkerSize', 10, ...
        'MarkerFaceColor', 'm');

    plot3(xMax, givenY, givenZ, 'ko', ...
        'MarkerSize', 10, ...
        'MarkerFaceColor', 'm');

    text(xMin, givenY, givenZ, '  xMin');
    text(xMax, givenY, givenZ, '  xMax');

    % Plot line between limits
    plot3([xMin, xMax], [givenY, givenY], [givenZ, givenZ], ...
        'm--', 'LineWidth', 2);
else
    fprintf('\nNo valid x-limits for given y and z.\n');
end
%}

%% ============================================================
%  SECTION 2: Find y-limits when x and z are known
%  Uncomment this section if you want y-limits.
%% ============================================================

%{
givenX = 17;  % cm
givenZ = 2.4;     % cm

[yMin, yMax, validY] = getYLimits(givenX, givenZ, R, r);

if validY
    fprintf('\nY-limit calculation:\n');
    fprintf('Given x = %.2f cm, z = %.2f cm\n', givenX, givenZ);
    fprintf('yMin = %.2f cm\n', yMin);
    fprintf('yMax = %.2f cm\n', yMax);

    % Plot y-limit points
    plot3(givenX, yMin, givenZ, 'ko', ...
        'MarkerSize', 10, ...
        'MarkerFaceColor', 'c');

    plot3(givenX, yMax, givenZ, 'ko', ...
        'MarkerSize', 10, ...
        'MarkerFaceColor', 'c');

    text(givenX, yMin, givenZ, '  yMin');
    text(givenX, yMax, givenZ, '  yMax');

    % Plot line between limits
    plot3([givenX, givenX], [yMin, yMax], [givenZ, givenZ], ...
        'c--', 'LineWidth', 2);
else
    fprintf('\nNo valid y-limits for given x and z.\n');
end
%}

%% ============================================================
%  SECTION 3: Find z-limit when x and y are known
%  Uncomment this section if you want z-limit.
%% ============================================================

%{
givenX = 14.5;  % cm
givenY = 0;     % cm

[zMin, zMax, validZ] = getZLimits(givenX, givenY, R, r);

if validZ
    fprintf('\nZ-limit calculation:\n');
    fprintf('Given x = %.2f cm, y = %.2f cm\n', givenX, givenY);
    fprintf('zMin = %.2f cm\n', zMin);
    fprintf('zMax = %.2f cm\n', zMax);

    % Plot z-limit points
    plot3(givenX, givenY, zMin, 'ko', ...
        'MarkerSize', 10, ...
        'MarkerFaceColor', 'y');

    plot3(givenX, givenY, zMax, 'ko', ...
        'MarkerSize', 10, ...
        'MarkerFaceColor', 'y');

    text(givenX, givenY, zMin, '  zMin');
    text(givenX, givenY, zMax, '  zMax');

    % Plot vertical line
    plot3([givenX, givenX], [givenY, givenY], [zMin, zMax], ...
        'y--', 'LineWidth', 2);
else
    fprintf('\nNo valid z-limits for given x and y.\n');
end
%}

%% Final Plot Settings

legend('Workspace boundary', ...
       'x-axis', 'y-axis', 'z-axis', ...
       'Location', 'best');

hold off;

%% ============================================================
%  Local Functions
%% ============================================================

function [xMin, xMax, valid] = getXLimits(y, z, R, r)
    % Finds valid x-limits for given y and z.
    % Workspace:
    % (sqrt(x^2 + y^2) - R)^2 + z^2 <= r^2
    % with x >= 0 and z >= 0.

    xMin = NaN;
    xMax = NaN;
    valid = false;

    % Basic top-half condition
    if z < 0 || z > r
        return;
    end

    s = sqrt(r^2 - z^2);

    rhoMin = R - s;
    rhoMax = R + s;

    % Outer limit
    outerVal = rhoMax^2 - y^2;

    if outerVal < 0
        return;
    end

    xMax = sqrt(outerVal);

    % Inner hole check
    innerVal = rhoMin^2 - y^2;

    if innerVal > 0
        xMin = sqrt(innerVal);
    else
        xMin = 0;
    end

    % Front-half condition
    if xMax < 0
        return;
    end

    valid = true;
end

function [yMin, yMax, valid] = getYLimits(x, z, R, r)
    % Finds valid y-limits for given x and z.
    % y can be positive or negative because y is side-to-side.
    % Workspace:
    % (sqrt(x^2 + y^2) - R)^2 + z^2 <= r^2
    % with x >= 0 and z >= 0.

    yMin = NaN;
    yMax = NaN;
    valid = false;

    % Basic front/top-half conditions
    if x < 0 || z < 0 || z > r
        return;
    end

    s = sqrt(r^2 - z^2);

    rhoMin = R - s;
    rhoMax = R + s;

    outerVal = rhoMax^2 - x^2;

    if outerVal < 0
        return;
    end

    yOuter = sqrt(outerVal);

    % Inner hole check
    innerVal = rhoMin^2 - x^2;

    if innerVal > 0
        % There is a forbidden gap around y = 0.
        % The true valid regions are:
        % -yOuter <= y <= -yInner
        % and
        %  yInner <= y <= yOuter
        %
        % For plotting the total extreme limits, return outer extremes.
        yMin = -yOuter;
        yMax = yOuter;

        fprintf('\nNote: inner torus hole creates forbidden y-region:\n');
        fprintf('Invalid region approximately: %.2f < y < %.2f\n', ...
            -sqrt(innerVal), sqrt(innerVal));
    else
        yMin = -yOuter;
        yMax = yOuter;
    end

    valid = true;
end

function [zMin, zMax, valid] = getZLimits(x, y, R, r)
    % Finds valid z-limits for given x and y.
    % Workspace:
    % (sqrt(x^2 + y^2) - R)^2 + z^2 <= r^2
    % with x >= 0 and z >= 0.

    zMin = NaN;
    zMax = NaN;
    valid = false;

    % Front-half condition
    if x < 0
        return;
    end

    rho = sqrt(x^2 + y^2);

    zVal = r^2 - (rho - R)^2;

    if zVal < 0
        return;
    end

    zMin = 0;
    zMax = sqrt(zVal);

    valid = true;
end