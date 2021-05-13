function [m,com,I,b,cob] = extractInertialPropertiesHull(data)

numel = length(data.tout);

m   = data.logsout.getElement('mass').Values.Data(numel);
com = data.logsout.getElement('CoM').Values.Data(numel,:);
I   = data.logsout.getElement('Inertia').Values.Data(:,:,numel);
b   = data.logsout.getElement('buoyancy').Values.Data(numel);
cob = data.logsout.getElement('CoB').Values.Data(numel,:);