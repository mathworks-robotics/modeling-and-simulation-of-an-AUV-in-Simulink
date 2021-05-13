
%setup (will require re-initialisation to modify properties)
ob = auvPathPlanner_MultiMode;
ob.occMap = 'UUVSceneLowResMap3.mat';
ob.maxRollAngle = pi/6; 
ob.airSpeed = 8;
ob.flightPathAngleLimit = [-0.1000 0.1000];
ob.bounds = [ -250, 250; -250, 250; -250, 250; -pi, pi ];
ob.searchCoordinates = [ 400, 400 ];
ob.searchFOV = 10;
ob.bouyPostions = [ -150, 150, -150, -150, 150, 0 ];
ob.bouyCoverage = 300;

%run plan (can be rerun using varied start/end/pathtype)
ob.step ([-150,-150,150,0],[-150,-150,50,0],3)
%optional over acutated path
%ob.step ([-150,-150,50,0],[-100,-100,120,0],2)


% %setup (will require re-initialisation to modify properties)
ob = auvPathPlanner_MultiMode;
ob.occMap = 'UUVSceneLowResMap3.mat';
ob.maxRollAngle = pi/3; 
ob.airSpeed = 10;
ob.flightPathAngleLimit = [-0.1000 0.1000];
ob.bounds = [ -250, 250; -250, 250; -250, 250; -pi, pi ];
ob.searchCoordinates = [ 400, 400 ];
ob.searchFOV = 10;
ob.bouyPostions = [ -150, 150, -150, -150, 150, 0 ];
ob.bouyCoverage = 300;
% 
% %run plan (can be rerun using varied start/end/pathtype)
ob.step ([-150,-150,150,0],[-150,-150,50,0],3)
% %optional over acutated path
% %ob.step ([-150,-150,50,0],[-100,-100,120,0],2)