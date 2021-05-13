% Read main propeller parameters 
J_vec = Vehicle.Prop.J_vec;
PDR   = Vehicle.Prop.pitchratio;
EAR   = Vehicle.Prop.propEAR;
Z     = Vehicle.Prop.numblades;

% Generate propeller thrust and torque coefficients
Coeff = generate_propcoefficients(J_vec,PDR,EAR,Z);

% Reuse figure if it exists, else create new figure
try
    figure(f1_AUV_PropCoefficients)
catch
    f1_AUV_PropCoefficients = ...
        figure('Name','f1_AUV_PropCoefficients');
end

% Set background color for the figure
set(gcf,'Color',[0.4,0.4,0.4])

% Plot propeller coefficients - use two separate scales for y axis
yyaxis left
plot(Coeff.Jt,Coeff.kt,'Color',[0 112 192]/255,'LineWidth',2)
hold on
plot(Coeff.Jt,Coeff.eta,'Color',[112 173 71]/255,'LineStyle','-','LineWidth',2)
set(gca,'XColor','w','YColor','w','GridColor',[0.4,0.4,0.4])
xlabel('$J = \frac{V_a}{ND}$','Interpreter','latex','Fontsize',14)
ylabel('k_t          \eta_o')
axis([0,1.4,0,1])
grid on
hold off

yyaxis right
plot(Coeff.Jq,Coeff.kq,'Color',[237 125 49]/255,'LineWidth',2)
set(gca,'YColor','w')
ylabel('k_q')
axis([0,1.4,0,0.2])
title('Propeller Efficiency, Thrust and Torque Coefficients','Color','w')
str1 = {'B-series Propeller Data'};
str2 = {'Z         = 7','P/D     = 1.24','A_e/A_o  = 0.54'};
text(0.15,0.16,str1,'Color',[0.4,0.4,0.4],'FontSize',12,...
    'FontWeight','bold','VerticalAlignment','bottom')
text(0.15,0.16,str2,'Color',[0.4,0.4,0.4],'FontSize',10,...
    'VerticalAlignment','top')
[~] = axtoolbar({'restoreview'});

legend({'k_t','\eta_o','k_q'},'Location','northeast')

clear Coeff J_vec PDR EAR Z str1 str2