% Initialization file for the AUV_ElectricalBattery_SPE model.
% This file load up the experimental data, as well as the initial
% guesses for the battery parameters on the model.

% Copyright 2020 The MathWorks, Inc.

%% Load experimental data and saved SPE session
load('AUV_BatteryData.mat');
load('AUV_ElectricBattery_SPE_spesession.mat');

%% Experiment duration
SimTime = batteryData.t(end);

%% Battery capacity
CAP = 31;                               % A*h
SOC_LUT = (0.1:.1:1);

%% Battery Parameters - Initial starting points before estimation
% Em open-circuit voltage vs SOC
Em_LUT = 3.8*ones(size(SOC_LUT));       % Volts

% R0 resistance vs SOC
R0_LUT = 0.01*ones(size(SOC_LUT));      % ohms

% Diffusion resistance
% R1 Resistance vs SOC
R1_LUT = 0.0005*ones(size(SOC_LUT));    % ohms

% tau1 Time Constant vs SOC
tau1_LUT = 10*ones(size(SOC_LUT));      % ohms

