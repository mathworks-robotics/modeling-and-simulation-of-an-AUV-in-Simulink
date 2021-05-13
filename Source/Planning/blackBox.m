function [pingLevelBB] = blackBox(poseBB,poseUUV,simulationTime)

% The function calculates the ping level of the Blackbox based on the distance and observation time from last ping.
% Inputs
% poseBB = [x y z], meters, position of the Blackbox
% poseUUV = [x y z heading], meters, position and heading of the UUV
% simulationTime = sec, simulation time in seconds
% Output
% pingLevelBB = %, Output is % of the signal level based on distance

pingInterval = 10; % Interval between pings
pingTime = mod(simulationTime,pingInterval); % How different is current time from the interval
pingBandwidth = 1; % How long the ping is audible after triggering
pingRange = 125;

distance = sqrt((poseBB(1) - poseUUV(1))^2 + (poseBB(2) - poseUUV(2))^2 + (poseBB(3) - poseUUV(3))^2);

if distance <= pingRange && pingTime < pingBandwidth
    pingLevelBB = (pingRange^2 - distance^2)/pingRange^2*100;
else
    pingLevelBB = 0;
end