% Code to run thruster force tests and plot the simulation results for
% the AUV_MechanicsHullJointsNForces model
% Copyright 2020 MathWorks, Inc.

% Store the name of the signal editor block
sigEditBlk = [gcs,'/Tunnel Thruster Forces (N)'];

% Uncomment signal editor block
set_param(sigEditBlk,'Commented','off');

% Configure simulation model for multiple test runs
set_param(gcs,'FastRestart','on','SimMechanicsOpenEditorOnUpdate','off');

% Determine the number of scenarios to run
numCases = str2double(get_param(sigEditBlk,'NumberOfScenarios'));

% Create the simulation input objects for the model (one object per sim)
for idx = numCases:-1:1
    in(idx) = Simulink.SimulationInput(gcs);
    in(idx) = setBlockParameter(in(idx), sigEditBlk, 'ActiveScenario', idx);
end

% Run simulations for ALL test scenarios
out = sim(in);

% Reuse figure if it exists, else create new figure
try
    figure(f1_AUV_MechanicsHull_Thrusters)
catch
    f1_AUV_MechanicsHull_Thrusters = ...
        figure('Name','f1_AUV_MechanicsHull_Thrusters');
end

% Extract and plot simulation results
for idx = 1:numCases
    
    simOut = out(idx);
    time = simOut.tout;
    wx = simOut.logsout_hullforces.getElement('wx').Values.Data;
    wy = simOut.logsout_hullforces.getElement('wy').Values.Data;
    wz = simOut.logsout_hullforces.getElement('wz').Values.Data;
    vx = simOut.logsout_hullforces.getElement('vx').Values.Data;
    vy = simOut.logsout_hullforces.getElement('vy').Values.Data;
    vz = simOut.logsout_hullforces.getElement('vz').Values.Data;
    
    subplot(321)
    plot(time,wx,'LineWidth',2)
    hold on
    axis auto
    
    subplot(323)
    plot(time,wy,'LineWidth',2)
    hold on
    axis auto
    
    subplot(325)
    plot(time,wz,'LineWidth',2)
    hold on
    axis auto
    
    subplot(322)
    plot(time,vx,'LineWidth',2)
    hold on
    axis auto
    
    subplot(324)
    plot(time,vy,'LineWidth',2)
    hold on
    axis auto
    
    subplot(326)
    plot(time,vz,'LineWidth',2)
    hold on
    axis auto
end

% Add the appropriate labels to each subplot
subplot(321)
hold off
title('Body Angular Velocity in X')
xlabel('time (sec)');
ylabel('wx (rad/s)');
box on
grid on

subplot(323)
hold off
title('Body Angular Velocity in Y')
xlabel('time (sec)');
ylabel('wy (rad/s)');
box on
grid on

subplot(325)
hold off
title('Body Angular Velocity in Z')
xlabel('time (sec)');
ylabel('wz (rad/s)');
box on
grid on

subplot(322)
hold off
title('Body Velocity in X')
xlabel('time (sec)');
ylabel('vx (m/s)');
box on
grid on

legend_array = {'Horizontal Thrusters ONLY',...
    'Horizontal and Vertical Thrusters',...
    'Vertical Thrusters ONLY'};
legend(legend_array,'Location','northwest')

subplot(324)
hold off
title('Body Velocity in Y')
xlabel('time (sec)');
ylabel('vy (m/s)');
box on
grid on

subplot(326)
hold off
title('Body Velocity in Z')
xlabel('time (sec)');
ylabel('vz (m/s)');
box on
grid on

% Set simulation model configuration back to original status
set_param(gcs,'FastRestart','off','SimMechanicsOpenEditorOnUpdate','on');
set_param(sigEditBlk,'Commented','on');

% Remove temporary variables from the workspace
clear simOut sigEditBlk numCases in out time wx wy wz vx vy vz
clear legend_array idx