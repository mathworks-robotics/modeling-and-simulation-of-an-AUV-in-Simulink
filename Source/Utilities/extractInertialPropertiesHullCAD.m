function [m,com,I] = extractInertialPropertiesHullCAD(data)

numel = length(data.tout);

m   = data.logsout.getElement('mass').Values.Data(numel);
com = data.logsout.getElement('CoM').Values.Data(numel,:);
I   = data.logsout.getElement('Inertia').Values.Data(:,:,numel);