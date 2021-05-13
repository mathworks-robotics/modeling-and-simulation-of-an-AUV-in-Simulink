function [pthObj,solnInfo,interpolatedPathObj,interpolatedSmoothWaypoints] = exampleExecutePlanner(planner,ss,sv,startPose,goalPose)
%exampleExecutePlanner creates the path and smooths it by calling needed
%functions

%   Copyright 2019 The MathWorks, Inc.
pthObj = navPath(ExampleHelperUAVStateSpace);

while(isempty(pthObj.States))
[pthObj,solnInfo] = plan(planner,startPose,goalPose);
end

interpolatedPathObj = copy(pthObj);
interpolate(interpolatedPathObj,200);

% smoothWaypointsObj = exampleHelperUAVPathSmoothing(ss,sv,pthObj);
if (pthObj.NumStates > 3)
smoothWaypointsObj = exampleHelperUAVPathSmoothingMod(ss,sv,pthObj);
interpolatedSmoothWaypoints = copy(smoothWaypointsObj);
else
interpolatedSmoothWaypoints = interpolatedPathObj;
end

%Commented out as if statement added
%smoothWaypointsObj = exampleHelperUAVPathSmoothingMod(ss,sv,pthObj); 
interpolate(interpolatedSmoothWaypoints,200);