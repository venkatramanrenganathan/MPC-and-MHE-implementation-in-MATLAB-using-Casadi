%
% This function draws a unicycle robot.
%
% Inputs:
%          param:  1x3 array
%           - state = [a, b, theta]
%           - (a,b): the center of the robot
%           - theta: the heading angle
%
% Output:
%          - hdl: plot handles
%
%
% (C) Kaveh Fathian, 2018.  Email: kaveh.fathian@gmail.com
%
function hdl = DrawUnicycle(state, scale, hdl)

if nargin == 2
    hdl = {[], [], []};
end

%%
x = state(1);
y = state(2);
theta = state(3);


%% Prameters
bodyWidth = 1*scale; 
bodyHeight = 1*scale;

wheelWidth = 0.5*scale;
wheelHeight = 0.15*scale;

bodyStyle.FaceColor = [255 99 0] ./ 255; % Orange
bodyStyle.EdgeColor = [70 35 10] ./ 255; % Light orange
bodyStyle.LineWidth = .10*scale;

wheelShift = (bodyHeight + wheelHeight) ./ 2; 
xWheel1 = x - wheelShift * sin(theta);
yWheel1 = y + wheelShift * cos(theta);
xWheel2 = x + wheelShift * sin(theta);
yWheel2 = y - wheelShift * cos(theta);

wheelStyle.FaceColor = [70 35 10] ./ 255; % Orange
wheelStyle.EdgeColor = [70 35 10] ./ 255; % Brown
wheelStyle.LineWidth = .1*scale;


%% Draw the robot

% Main body
param = [x,y, bodyWidth, bodyHeight, theta];
hb = DrawRectangle(param, bodyStyle, hdl{1});

% Wheels
param = [xWheel1, yWheel1, wheelWidth, wheelHeight, theta];
hw1 = DrawRectangle(param, wheelStyle, hdl{2});
param = [xWheel2, yWheel2, wheelWidth, wheelHeight, theta];
hw2 = DrawRectangle(param, wheelStyle, hdl{3});

hdl = {hb, hw1, hw2};

end