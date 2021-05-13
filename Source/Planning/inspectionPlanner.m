function [xCoord,yCoord,zCoord] = inspectionPlanner(mapX,mapY,robotPosX,robotPosY,robotPosZ,FOV)
%% define environment
% envmap = zeros(100,100);
envmap = zeros(mapX,mapY);
[m,n] = size(envmap);
% envmap(57:96,49:87) = 1;

robotPos = [1,1];
% virtualgoalPos = [-1000 150];
virtualgoalPos = [1000 150];
envmap(robotPos(1),robotPos(2)) = 2;

%% plan
if (~exist ('AStar.mexw64','file'))
mex AStar.cpp;
end
% FOV = 60; % assume onboard camera has square FOV

xPos = [robotPos(1)];
yPos = [robotPos(2)];

% dX = [-1, -1, -1,  0,  0,  1, 1, 1];
% dY = [-1,  0,  1, -1,  1, -1, 0, 1];
% N = length(dX);

for i =1:mapX*mapY % 1025
    action = AStar(envmap, robotPos, virtualgoalPos);
    action = int64(action);
    if (action(1)==0 && action(2)==0)
%         disp("Finished coverage.");
        break;
    end
    
    robotPos(1) = robotPos(1)+action(1);
    robotPos(2) = robotPos(2)+action(2);
    envmap(robotPos(1),robotPos(2)) = 2;
    
    xPos = [xPos, robotPos(1)];
    yPos = [yPos, robotPos(2)];    
    
    % update map to set explored for all cells in FOV
%     for x= robotPos(1)-FOV: robotPos(1)+FOV
%         for y = robotPos(2)-FOV: robotPos(2)+FOV
%             if x>0 && y>0 && x<=m && y<=n
%                 envmap(x,y) = 2;
%             end
%         end
%     end

envmap(max(1,robotPos(1)-FOV):min(m,robotPos(1)+FOV),max(1,robotPos(2)-FOV):min(n,robotPos(2)+FOV)) = 2;
end


len = size(xPos,2);
xCoord = [robotPosX robotPosX + xPos - 1];
yCoord = [robotPosY robotPosY + yPos - 1];
zCoord = [robotPosZ robotPosZ*ones(1,len)];

% % visualize path
% figure
% hold on
% imshow((envmap)'*256,'InitialMagnification','fit');
% 
% for i = 1:m
%     for j = 1:n
%         xLeft = i - 1/2;
%         yBottom = j - 1/2;
%         if envmap(i,j)==2
%             c = 'y';
%         elseif envmap(i,j)==1
%             c = 'k';
%         else
%             c = 'w';
%         end
%         
%         rectangle('Position', [xLeft, yBottom, 1, 1], 'EdgeColor', c, 'FaceColor', c, 'LineWidth', 1);
%     end
% end
% % 
% % line(xPos,yPos,'Color','red','LineWidth',2);
% % plot(robotPos(1), robotPos(2),'ro','MarkerFaceColor','r','MarkerSize',6)
% % plot(startPos(1), startPos(2),'bo','MarkerFaceColor','b','MarkerSize',6)
% 
% axes = gca;
% axes.Visible = 'On';
% axes.YDir = 'normal';
% xlim([0,m]);
% ylim([0,n]);


