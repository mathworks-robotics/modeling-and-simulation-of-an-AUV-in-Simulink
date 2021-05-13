% Code to test the vehicle variants and plot the simulation results for
% the AUV_MechanicsFullHydrodynamicsVAR model
% Copyright 2020 MathWorks, Inc.

% Disable model pacing
set_param(gcs,'EnablePacing','off')

% Turn off the Mechanics Explorer
set_param(gcs,'SimMechanicsOpenEditorOnUpdate','off');

% Set the variant block
variantBlk = 'AUV_MechanicsFullHydrodynamicsVAR/AUV';
actvar = {'Multibody','Singlebody'};
style  = {'-','-.'};

% Create the simulation input objects for the model (one object per sim)
for idx = 2:-1:1
    in(idx) = Simulink.SimulationInput(gcs);
    in(idx) = setBlockParameter(in(idx),variantBlk,'VariantLabel',actvar{idx});
end

% Run simulations for ALL test scenarios
out = sim(in);

% Reuse figure if it exists, else create new figure
try
    figure(f1_AUV_MechanicsFullHydrodynamicsVAR)
catch
    f1_AUV_MechanicsFullHydrodynamicsVAR = ...
        figure('Name','f1_AUV_MechanicsFullHydrodynamicsVAR');
end

% Set background color for the figure
set(gcf,'Color',[0.4,0.4,0.4])

% Extract and plot simulation results
for idx = 1:2
    
    simOut = out(idx);
    time   = simOut.tout;
    speed  = simOut.logsout_hydrodynamicsVAR.getElement('speed (knots)').Values.Data;
    phi    = simOut.logsout_hydrodynamicsVAR.getElement('roll,pitch (deg)').Values.Data(:,1);
    theta  = simOut.logsout_hydrodynamicsVAR.getElement('roll,pitch (deg)').Values.Data(:,2);
    psi    = simOut.logsout_hydrodynamicsVAR.getElement('yaw (deg)').Values.Data;
    
    subplot(221)
    plot(time,speed,'LineWidth',2,'LineStyle',style{idx})
    hold on
    axis auto
    
    subplot(222)
    plot(time,phi,'LineWidth',2,'LineStyle',style{idx})
    hold on
    axis auto
    
    subplot(223)
    plot(time,theta,'LineWidth',2,'LineStyle',style{idx})
    hold on
    axis auto
    
    subplot(224)
    plot(time,psi,'LineWidth',2,'LineStyle',style{idx})
    hold on
    axis auto
end

% Add the appropriate labels to each subplot
subplot(221)
set(gca,'XColor','w','YColor','w','GridColor',[0.4,0.4,0.4])
hold off
title('Vehicle Speed','Color','w')
xlabel('time (sec)');
ylabel('V (kt)');
box on
grid on
legend(actvar,'Location','southeast')
[~] = axtoolbar({'restoreview'});

subplot(222)
set(gca,'XColor','w','YColor','w','GridColor',[0.4,0.4,0.4])
hold off
title('Roll Angle','Color','w')
xlabel('time (sec)');
ylabel('\phi (deg)');
box on
grid on
legend(actvar,'Location','southwest')
[~] = axtoolbar({'restoreview'});

subplot(223)
set(gca,'XColor','w','YColor','w','GridColor',[0.4,0.4,0.4])
hold off
title('Pitch Angle','Color','w')
xlabel('time (sec)');
ylabel('\theta (deg)');
box on
grid on
legend(actvar,'Location','northeast')
[~] = axtoolbar({'restoreview'});

subplot(224)
set(gca,'XColor','w','YColor','w','GridColor',[0.4,0.4,0.4])
hold off
title('Yaw Angle','Color','w')
xlabel('time (sec)');
ylabel('\psi (deg)');
box on
grid on
legend(actvar,'Location','northwest')
[~] = axtoolbar({'restoreview'});

% Set simulation model configuration back to original status
set_param(gcs,'SimMechanicsOpenEditorOnUpdate','on','EnablePacing','on');

% Remove temporary variables from the workspace
clear simOut variantBlk actvar in out idx time
clear speed phi theta psi style