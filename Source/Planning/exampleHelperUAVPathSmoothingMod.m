function [smoothWaypointsObj] = exampleHelperUAVPathSmoothingMod(ss,sv,pthObj)
%exampleHelperUAVPathSmoothing Smooth Dubins path iteratively

%   Copyright 2019 The MathWorks, Inc.

%removes intermediate waypoints between two distant poses
%if this does not result in a collision with the environment.

nonSmoothWaypoints=pthObj.States(:,1:size(pthObj.States,2));
counter=1;
optimizedWaypoints(counter,:)=nonSmoothWaypoints(1,:);
startNode=1;
endNode=startNode+1;
counter=counter+1;
lastNonCollisionNode=endNode;
while(endNode<=length(nonSmoothWaypoints))
    MotionValid=isMotionValid(sv,nonSmoothWaypoints(startNode,:),nonSmoothWaypoints(endNode,:));
    collide=~MotionValid;
    if(~collide)
        optimizedWaypoints(counter,:)=nonSmoothWaypoints(endNode,:);
        lastNonCollisionNode=endNode;
        endNode=endNode+1;
    end
    if(collide)
        breakcheck = 0;
        for endNodeCounter = length(nonSmoothWaypoints):-1:endNode+1
            MotionValidCollide=isMotionValid(sv,nonSmoothWaypoints(startNode,:),nonSmoothWaypoints(endNodeCounter,:));
            collideCollide=~MotionValidCollide;
            if (~collideCollide)
                optimizedWaypoints(counter,:)=nonSmoothWaypoints(endNodeCounter,:);
                lastNonCollisionNode=endNodeCounter;
                counter=counter+1;
                startNode=lastNonCollisionNode;
                endNode=startNode+1;
                breakcheck = 1;
                break
            end
        end
        if breakcheck == 0
            optimizedWaypoints(counter,:)=nonSmoothWaypoints(lastNonCollisionNode,:);
            counter=counter+1;
            startNode=lastNonCollisionNode;
            endNode=startNode+1;
        end
    end
end
%define an empty navPath Object.
smoothWaypointsObj=navPath(ss);
%add smooth waypoints to the object.
append(smoothWaypointsObj, optimizedWaypoints(:,1:size(pthObj.States,2)));

end

