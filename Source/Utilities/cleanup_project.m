% This is a utility script that runs automatically when the project
% is closed

% Close any open Simulink models and clear the Simulation Data Inspector
bdclose('all');
Simulink.sdi.clear;

% Close any open figures
close all;

% Clear workspace
clear;