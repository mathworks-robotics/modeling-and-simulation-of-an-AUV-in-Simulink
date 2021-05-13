function [triangulationTrigger,triangulationHeading,poseEstimateBB] = triangulationPlanner(poseBB,poseUUV,simulationTime,triangulationTrigger,triangulationHeading)
% The function triangulates the position of the blackbox based on 2 pings being heard.
% Inputs
% poseBB = [x y z], meters, position of the Blackbox
% poseUUV = [x y z heading], meters, position and heading of the UUV
% simulationTime = sec, simulation time in seconds
% triangulationTrigger = 0 to 3, this notes down what stage of triangulation this is at, THIS VARIABLE NEEDS TO BE INITIALIZED AT THE START OF THE CODE CALLING THIS FUNCTION AS triangulationTrigger = 0
% triangulationHeading = array with a row of 7, this variable notes down the history of pings recorded, THIS VARIABLE NEEDS TO BE INITIALIZED AT THE START OF THE CODE CALLING THIS FUNCTION AS triangulationHeading = [NaN NaN NaN NaN NaN NaN NaN]
% Output
% triangulationTrigger = 0 to 3, this notes down what stage of triangulation this is at, 0: No pings heard, 1: 1 ping heard and recorded, 2: second ping not on the same heading as the previous ping or within a treshold of the old ping location heard, 3: Triangulation completed
% triangulationHeading = array with a row of 7, this variable notes down the history of pings recorded
% poseEstimateBB = [x y z errorRange], position of the blackbox (Currently errorRnage is 3D error) and errorRange based on the phased array sonar accuracy

poseEstimateBB = [NaN NaN NaN NaN];
minSecondReadingDist = 10; % Minimum distance UUV should be from the previous reading to get a new reading
minSecondReadingHeadingDiff = 5/180*pi(); % Minimum heading angle difference UUV should have from previous reading to get a new reading

[pingTrigger,triangulationHeadingNew] = phasedArraySonar(poseBB,poseUUV,simulationTime,0);

if pingTrigger == 1
    if triangulationTrigger == 0
        triangulationHeading(1,:) = triangulationHeadingNew;
        triangulationTrigger = 1;
    elseif triangulationTrigger == 1 && abs(triangulationHeading(1,1) - triangulationHeadingNew(1)) > minSecondReadingHeadingDiff && abs(abs(triangulationHeading(1,1) - triangulationHeadingNew(1)) - pi()) > minSecondReadingHeadingDiff && sqrt((poseUUV(1) - triangulationHeading(1,4))^2 + (poseUUV(2) - triangulationHeading(1,5))^2 + (poseUUV(3) - triangulationHeading(1,6))^2) > minSecondReadingDist
        triangulationHeading(2,:) = triangulationHeadingNew;
        triangulationTrigger = 2;
    end
end

if triangulationTrigger == 2
    lineSlope = tan(pi()/2 - triangulationHeading(:,1));
    lineIntercept = triangulationHeading(:,5) - lineSlope.*triangulationHeading(:,4);
    poseEstimateBB(1) = (lineIntercept(2) - lineIntercept(1))/(lineSlope(1)-lineSlope(2));
    poseEstimateBB(2) = lineSlope(1)*poseEstimateBB(1) + lineIntercept(1);
    poseEstimateBB(3) = triangulationHeading(2,6) + tan(triangulationHeading(2,2))*sqrt((poseEstimateBB(1) - triangulationHeading(2,4))^2 + (poseEstimateBB(2) - triangulationHeading(2,5))^2);
    poseEstimateBB(4) = 2*sqrt((poseEstimateBB(1) - triangulationHeading(2,4))^2 + (poseEstimateBB(2) - triangulationHeading(2,5))^2 + (poseEstimateBB(3) - triangulationHeading(2,6))^2)*tan(triangulationHeading(2,3));
    triangulationTrigger = 3;
end
    