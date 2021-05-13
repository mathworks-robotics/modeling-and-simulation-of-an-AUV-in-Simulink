% Code to plot simulation results from AUV_ControlsPreliminary model
% Copyright 2021 MathWorks, Inc.

% Reuse figure if it exists, else create new figure
try
    figure(f1_controlspreliminary)
catch
    f1_controlspreliminary = figure('Name','f1_controlspreliminary');
end

% Set background color for the figure
set(gcf,'Color',[0.4,0.4,0.4])

% Generate simulation results if they don't exist
if(~exist('out','var'))
    out = sim(gcs);
end

% Get simulation results
DataX = out.logsout_controls{1}.Values.CoM.World.Xe.X.Data;
DataY = out.logsout_controls{1}.Values.CoM.World.Xe.Y.Data;
DataZ = out.logsout_controls{1}.Values.CoM.World.Xe.Z.Data;

% Plot results
plot3(DataX,DataY,DataZ,'LineWidth',2,'Color',[0.93,0.69,0.13]);
set(gca,'XColor','w','YColor','w','ZColor','w','GridColor',[0.4,0.4,0.4])
grid on
axis equal
title('AUV Trajectory','Color','w');
xlabel('X (m)','Color','w');
ylabel('Y (m)','Color','w');
zlabel('Z (m)','Color','w');

% Remove temporary variables
clear DataX DataY DataZ
