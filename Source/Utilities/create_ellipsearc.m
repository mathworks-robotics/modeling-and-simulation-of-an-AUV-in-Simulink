function [x,y] = create_ellipsearc(center,a_axis,b_axis,s_angle,e_angle)
% create_ellipsearc
% This function creates an ellipse arc profile
%
% The usage of the function is:
%   [x,y] = create_ellipsearc(center,diam,s_angle,e_angle)
% The inputs are:
%   center = (x,y) coordinate pair for the center of the ellipse
%   a_axis = major axis of the ellipse
%   b_axis = minor axis of the ellipse
%   s_angle = start angle for the arc (deg)
%   e_angle = end angle for the arc (deg)
% The outputs x and y are vectors of coordinate points
% for the resulting ellipse arc.
%

% Copyright 2020 The MathWorks, Inc.

% Convert angles to radians
st = s_angle*pi/180;
en = e_angle*pi/180;
% Define the angle range of the arc
r_angle = (st:(en-st)/99:en)';

% Calculate x and y
x = center(1) + a_axis*cos(r_angle);
y = center(2) + b_axis*sin(r_angle);

end