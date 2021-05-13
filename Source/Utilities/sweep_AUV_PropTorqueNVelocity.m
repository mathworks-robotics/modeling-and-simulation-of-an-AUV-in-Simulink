% Code to run a parameter sweep and plot simulation results from
% AUV_ElectroMechanicsFullPropeller model
% Copyright 2020 MathWorks, Inc.

% Disable the Mechanical Explorer visualization
set_param(gcs,'SimMechanicsOpenEditorOnUpdate','off');

% Save changes to the model (for parallel simulations)
save_system(gcs);

% Set sweep range for the reference torque value (N*m)
Tref_sweep = 0.2:0.025:0.5;
% Set sweep range for inflow velocity value
Va_sweep = 0:0.25:3;
% Create a combined parameter grid for the sweep
[TT,VV] = meshgrid(Tref_sweep,Va_sweep);

% Determine the total number of simulations to run
numSims = numel(TT);

% Create the simulation input objects for the model (one object per sim)
for i = numSims:-1:1
    in(i) = Simulink.SimulationInput(gcs);
    in(i) = setBlockParameter(in(i),...
        [gcs '/Torque Reference'],'Value',num2str(TT(i)));
    in(i) = setBlockParameter(in(i),...
        [gcs '/Inflow Velocity'],'Value',num2str(VV(i)));
end

% Run parallel simulations
out = parsim(in,'ShowSimulationManager','on','UseFastRestart','on',...
    'ShowProgress','off');

% Extract final (steady state) values from simulation results
for i = numSims:-1:1
    simOut = out(i);
    thrust(i) = simOut.logsout_emFullprop.getElement('thrust').Values.Data(end);
    torque(i) = simOut.logsout_emFullprop.getElement('torque').Values.Data(end);
    speed(i)  = simOut.logsout_emFullprop.getElement('speed' ).Values.Data(end);
    legend_array{i} = ['V_a = ',num2str(VV(i))];
end

% Reshape steady state value vectors into arrays
thrust = reshape(thrust,[length(Va_sweep),length(Tref_sweep)])';
torque = reshape(torque,[length(Va_sweep),length(Tref_sweep)])';
speed  = reshape(speed ,[length(Va_sweep),length(Tref_sweep)])';

% Reshape legend array
legend_array = reshape(legend_array,[length(Va_sweep),length(Tref_sweep)])';
legend_array = legend_array(1,:);

% Unit conversion
speedRPM = speed*30/pi;

% Reuse figure if it exists, else create new figure
try
    figure(f1_emFullPropeller)
catch
    f1_emFullPropeller = ...
        figure('Name','emFullPropeller');
end

% Set the figure size
f1_emFullPropeller.Position = [240,144,960,520];

% Plot the results
subplot(211)
plot(speedRPM(:,1:2:end),thrust(:,1:2:end),'LineWidth',2)
xlabel('RPM')
ylabel('Thrust (N)')
title('Propeller Thrust vs RPM @ Steady State')
grid on
legend(legend_array(1:2:end),'Location','northeast')
subplot(212)
plot(speedRPM(:,1:2:end),torque(:,1:2:end),'LineWidth',2)
xlabel('RPM')
ylabel('Torque (N*m)')
title('Propeller Torque vs RPM @ Steady State')
grid on

% Set simulation model configuration back to original status
set_param(gcs,'SimMechanicsOpenEditorOnUpdate','on');
set_param([gcs '/Torque Reference'],'Value','0.3');
set_param([gcs '/Inflow Velocity'],'Value','2');
% Save changes to the model
save_system(gcs);

% Remove temporary variables
clear Tref_sweep Va_sweep TT VV numSims i
clear in simOut thrust torque speed speedRPM legend_array