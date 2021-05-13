% Load battery final estimation parameters
load('AUV_BatteryParametersFinal.mat');

% Reuse figure if it exists, else create new figure
try
    figure(f1_AUV_BatteryParameters)
catch
    f1_AUV_BatteryParameters = ...
        figure('Name','f1_AUV_BatteryParameters');
end

% Set background color for the figure
set(gcf,'Color',[0.4,0.4,0.4])

% Plot battery final parameters versus SOC
subplot(221)
plot(SOC_LUT,Em_LUT,'Color',[210,120,9]/255,'LineWidth',2)
set(gca,'XColor','w','YColor','w','GridColor',[0.4,0.4,0.4])
xlabel('SOC')
ylabel('Volts')
title('Open Circuit Voltage - Em','Color','w')
grid on
[~] = axtoolbar({'restoreview'});

subplot(222)
plot(SOC_LUT,R0_LUT,'Color',[210,120,9]/255,'LineWidth',2)
set(gca,'XColor','w','YColor','w','GridColor',[0.4,0.4,0.4])
xlabel('SOC')
ylabel('ohms')
title('Internal Resistance - R0','Color','w')
grid on
[~] = axtoolbar({'restoreview'});

subplot(223)
plot(SOC_LUT,R1_LUT,'Color',[210,120,9]/255,'LineWidth',2)
set(gca,'XColor','w','YColor','w','GridColor',[0.4,0.4,0.4])
xlabel('SOC')
ylabel('ohms')
title('Diffusion Resistance - R1','Color','w')
grid on
[~] = axtoolbar({'restoreview'});

subplot(224)
plot(SOC_LUT,tau1_LUT,'Color',[210,120,9]/255,'LineWidth',2)
set(gca,'XColor','w','YColor','w','GridColor',[0.4,0.4,0.4])
xlabel('SOC')
ylabel('sec')
title('Time Constant - Tau1','Color','w')
grid on
[~] = axtoolbar({'restoreview'});
