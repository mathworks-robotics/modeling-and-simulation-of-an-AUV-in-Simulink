% Define ranges for alpha and beta
alpha_vec = Vehicle.IncRange.alpha_vec;
beta_vec = Vehicle.IncRange.beta_vec;

% Generate hydrodynamic coefficients
Coeff = generate_hydrocoefficients(alpha_vec,beta_vec);

% Reuse figure if it exists, else create new figure
try
    figure(f1_AUV_HydroCoefficients)
catch
    f1_AUV_HydroCoefficients = ...
        figure('Name','f1_AUV_HydroCoefficients');
end

% Set background color for the figure
set(gcf,'Color',[0.4,0.4,0.4])

% Plot all coefficient surface maps
subplot(321)
surf(alpha_vec,beta_vec,Coeff.CX,'EdgeAlpha',0.25)
set(gca,'XColor','w','YColor','w','ZColor','w','GridColor',[0.4,0.4,0.4])
xlabel('alpha','Color','w')
ylabel('beta','Color','w')
title('CX','Color','w')
[~] = axtoolbar({'rotate','restoreview'});

subplot(323)
surf(alpha_vec,beta_vec,Coeff.CY,'EdgeAlpha',0.25)
set(gca,'XColor','w','YColor','w','ZColor','w','GridColor',[0.4,0.4,0.4])
xlabel('alpha','Color','w')
ylabel('beta','Color','w')
title('CY','Color','w')
[~] = axtoolbar({'rotate','restoreview'});

subplot(325)
surf(alpha_vec,beta_vec,Coeff.CZ,'EdgeAlpha',0.25)
set(gca,'XColor','w','YColor','w','ZColor','w','GridColor',[0.4,0.4,0.4])
xlabel('alpha','Color','w')
ylabel('beta','Color','w')
title('CZ','Color','w')
[~] = axtoolbar({'rotate','restoreview'});

subplot(322)
surf(alpha_vec,beta_vec,Coeff.CMK,'EdgeAlpha',0.25)
set(gca,'XColor','w','YColor','w','ZColor','w','GridColor',[0.4,0.4,0.4])
xlabel('alpha','Color','w')
ylabel('beta','Color','w')
title('CMK','Color','w')
[~] = axtoolbar({'rotate','restoreview'});

subplot(324)
surf(alpha_vec,beta_vec,Coeff.CMM,'EdgeAlpha',0.25)
set(gca,'XColor','w','YColor','w','ZColor','w','GridColor',[0.4,0.4,0.4])
xlabel('alpha','Color','w')
ylabel('beta','Color','w')
title('CMM','Color','w')
[~] = axtoolbar({'rotate','restoreview'});

subplot(326)
surf(alpha_vec,beta_vec,Coeff.CMN,'EdgeAlpha',0.25)
set(gca,'XColor','w','YColor','w','ZColor','w','GridColor',[0.4,0.4,0.4])
xlabel('alpha','Color','w')
ylabel('beta','Color','w')
title('CMN','Color','w')
[~] = axtoolbar({'rotate','restoreview'});

clear Coeff alpha_vec beta_vec