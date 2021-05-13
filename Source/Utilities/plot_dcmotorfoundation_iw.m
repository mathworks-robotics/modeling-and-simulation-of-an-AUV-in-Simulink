% Code to plot simulation results from AUV_ElectricalDCmotorFoundation model
% Copyright 2020 MathWorks, Inc.

% Reuse figure if it exists, else create new figure
try
    figure(f1_dcmotorfoundation)
catch
    f1_dcmotorfoundation = figure('Name','f1_dcmotorfoundation');
end

% Generate simulation results if they don't exist
if(~exist('out','var'))
    out = sim(gcs);
end

% Get simulation results
current = out.simlog_dcmotorfoundation.R.i.series;
speed   = out.simlog_dcmotorfoundation.J.w.series;

% Plot results
ah(1) = subplot(211);
plot(current.time,current.values,'LineWidth',1);
grid on
title('Motor Current');
ylabel('Current (A)');
ah(2) = subplot(212);
plot(speed.time,speed.values,'LineWidth',1);
grid on
title('Motor Speed');
xlabel('Time (s)');
ylabel('Speed (rad/s)');

linkaxes(ah,'x');

% Remove temporary variables
clear current speed ah
