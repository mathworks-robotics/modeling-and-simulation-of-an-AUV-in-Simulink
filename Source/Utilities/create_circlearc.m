function [x,y] = create_circlearc(center,diam,s_angle,e_angle)
% create_circlearc
% This function creates a circle arc profile
%
% The usage of the function is:
%   [x,y] = create_circlearc(center,diam,s_angle,e_angle)
% The inputs are:
%   center = (x,y) coordinate pair for the center of the circle
%   diam = diameter of the circle
%   s_angle = start angle for the arc (deg)
%   e_angle = end angle for the arc (deg)
% The outputs x and y are vectors of coordinate points
% for the resulting circle arc.
%

% Copyright 2020 The MathWorks, Inc.

% Convert angles to radians
st = s_angle*pi/180;
en = e_angle*pi/180;
% Define the angle range of the arc
r_angle = (st:(en-st)/99:en)';

% Calculate x and y
x = center(1) + diam/2*cos(r_angle);
y = center(2) + diam/2*sin(r_angle);

end