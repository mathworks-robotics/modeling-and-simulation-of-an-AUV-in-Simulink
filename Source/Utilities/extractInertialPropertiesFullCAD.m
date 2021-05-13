function [b,cob,m,com,I] = extractInertialPropertiesFullCAD(data)

numel = length(data.tout);

b   = data.logsout.getElement('buoyancy').Values.Data(numel);
cob = data.logsout.getElement('CoB').Values.Data(numel,:);
m   = data.logsout.getElement('mass').Values.Data(numel);
com = data.logsout.getElement('CoM').Values.Data(numel,:);
I   = data.logsout.getElement('Inertia').Values.Data(:,:,numel);