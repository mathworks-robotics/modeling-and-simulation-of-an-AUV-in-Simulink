function coeff = generate_propcoefficients(J,PDR,EAR,Z)
% GENERATE PROPCOEFFICIENTS
% This function calculates the values of the thrust and torque coefficients
% kt and kq for a propeller as a function of the advance velocity ratio,
% pitch/diameter ratio, expanded area ratio and number of blades.
%
% Inputs:
% J   : Advance velocity ratio (Va/N*D)
% PDR : Pitch/diameter ratio (P/D)
% EAR : Expanded area ratio (Ae/Ao)
% Z   : Number of propeller blades
%
% Outputs:
% coeff.kt  : Thrust coefficient
% coeff.kq  : Torque coefficient
% coeff.eta : Open water efficiency

% Load polynomial fit coefficients (B-series Propeller Efficiency Curves)
load 'AUV_PropPolyCoeff.mat' CTstuv st tt ut vt CQstuv sq tq uq vq;

% Evaluate kt and kq for all values of J
kt = sum(CTstuv.*J.^st.*PDR.^tt.*EAR.^ut.*Z.^vt);
kq = sum(CQstuv.*J.^sq.*PDR.^tq.*EAR.^uq.*Z.^vq);

% Keep only positive values of kt and kq and match corresponding vector
% sizes for J
coeff.kt = kt(kt >= 0);
coeff.kq = kq(kq >= 0);
coeff.Jt = J(1:length(coeff.kt));
coeff.Jq = J(1:length(coeff.kq));

% Evaluate efficiency eta
coeff.eta = coeff.Jt/2/pi.*coeff.kt./coeff.kq(1:length(coeff.kt));

