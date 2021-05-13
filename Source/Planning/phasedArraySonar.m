function [pingTrigger,headingBB] = phasedArraySonar(poseBB,poseUUV,simulationTime,deg)

% The function calculates if the phased sonar array heard the ping from the blackbox and heading of the signal.
% Inputs
% poseBB = [x y z], meters, position of the Blackbox
% poseUUV = [x y z heading], meters, position and heading of the UUV
% simulationTime = sec, simulation time in seconds
% deg = 0 or 1, 0 sets the output to radians, 1 sets the output to degrees
% Output
% pingTrigger = 0 or 1, 0 if no ping was detected from blackbox, 1 if a ping was detected
% headingBB = 0 to 2pi radians or 0 to 360 degrees, heading from which the ping was heard

pingTrigger = 0;
if deg ~= 1
    phasedarrayAccuracy = 15/180*pi(); % Phased Array +- accuracy in azumith radians
else
    phasedarrayAccuracy = 7.5; % Phased Array +- accuracy in azumith degrees
end
headingBB = [NaN NaN NaN NaN NaN NaN NaN];

pingLevelBB = blackBox(poseBB,poseUUV,simulationTime);

if pingLevelBB > 0
    pingTrigger = 1;
    if deg ~= 1
        headingBB(1) = atan2((poseBB(1) - poseUUV(1)),(poseBB(2) - poseUUV(2))); % For radians
        if headingBB(1) <= 0
            headingBB(1) = headingBB(1) + 2*pi(); % For radians
        end
        headingBB(2) =  atan((poseBB(3) - poseUUV(3))/sqrt((poseBB(1) - poseUUV(1))^2 + (poseBB(2) - poseUUV(2))^2));
        headingBB(3) = phasedarrayAccuracy;
        headingBB(4:7) = poseUUV;
    else
        headingBB(1) = atan2d((poseBB(1) - poseUUV(1)),(poseBB(2) - poseUUV(2)));
        if headingBB(1) <= 0
            headingBB(1) = headingBB(1) + 360;
        end
        headingBB(2) =  atand((poseBB(3) - poseUUV(3))/sqrt((poseBB(1) - poseUUV(1))^2 + (poseBB(2) - poseUUV(2))^2));
        headingBB(3) = phasedarrayAccuracy;
        headingBB(4:7) = poseUUV;
        headingBB(7) = headingBB(7)/pi()*180;
    end
end
