% Define ranges for alpha
alpha_vec_damp = Vehicle.IncRange.alpha_vec_damp;

% Generate body rate damping coefficients
Damp = generate_dampcoefficients(alpha_vec_damp);

% Reuse figure if it exists, else create new figure
try
    figure(f1_AUV_DampCoefficients)
catch
    f1_AUV_DampCoefficients = ...
        figure('Name','f1_AUV_DampCoefficients');
end

% Set background color for the figure
set(gcf,'Color',[0.4,0.4,0.4])

% Plot body rate damping coefficients
subplot(221)
plot(alpha_vec_damp,Damp.Ckp,'r',alpha_vec_damp,Damp.Cnp,'b','LineWidth',1)
set(gca,'XColor','w','YColor','w','GridColor',[0.4,0.4,0.4])
xlabel('alpha (deg)')
title('Clp,Cnp','Color','w')
legend({'Ckp','Cnp'},'location','SouthWest')
grid on
[~] = axtoolbar({'restoreview'});

subplot(212)
plot(alpha_vec_damp,Damp.Cmq,'g','LineWidth',1)
set(gca,'XColor','w','YColor','w','GridColor',[0.4,0.4,0.4])
xlabel('alpha (deg)')
title('Cmq','Color','w')
legend({'Cmq'},'location','NorthEast')
grid on
[~] = axtoolbar({'restoreview'});

subplot(222)
plot(alpha_vec_damp,Damp.Ckr,'r',alpha_vec_damp,Damp.Cnr,'b','LineWidth',1)
set(gca,'XColor','w','YColor','w','GridColor',[0.4,0.4,0.4])
xlabel('alpha (deg)')
title('Clr, Cnr','Color','w')
legend({'Ckr','Cnr'},'location','NorthWest')
grid on
[~] = axtoolbar({'restoreview'});

clear Damp alpha_vec_damp