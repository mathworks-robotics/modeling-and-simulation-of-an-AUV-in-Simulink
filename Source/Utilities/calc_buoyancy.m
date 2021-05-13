function [b,cob,v] = calc_buoyancy(dhull,lhull,lsleeve,dmotor,ltunnel,...
    dtunnel,lbowcap,lsterncap,thick,rho)
% BUOYANCY
% This function calculates the approximate total volume of fluid
% displaced by the vehicle body when fully submerged and the buoyancy
% of the vehicle in that fluid.

% Calculate the displaced fluid volumes corresponding to each of the
% main sections of the vehicle (assumming the vehicle is fully submerged)

% Copyright 2020 The MathWorks, Inc.

% Main hull section
vhull = lhull*pi*dhull^2/4;                         % cylindrical body
% Center sleeve and motor mount
vsleeve = lsleeve*pi*((dhull+thick)^2-dhull^2)/4;   % cylindrical tube
vwing = lsleeve*lsleeve/2*thick/2;                  % brick body
vmotormount = lsleeve*pi*(dmotor+thick)^2/4;        % cylindrical body
vgpsmount = lsleeve*5*thick*3*thick/2;              % brick body
% Tunnel thruster section
vtunnel = ltunnel*pi*dhull^2/4;                     % cylindrical body
vtunnelhole = dhull*pi*dtunnel^2/4;                 % cylindrical hole
% Bow cap
vbowcap = 4*pi*lbowcap*(dhull/2)^2/3/2;             % half ellipsoid
% Stern cap
vsterncap = 4*pi*lsterncap*(dhull/2)^2/3/2;         % half ellipsoid

% Calculate the total displaced fluid volume and corresponding bouyancy
v = vhull + vsleeve + 2*vwing + 2*vmotormount + vgpsmount +...
    4*vtunnel - 4*vtunnelhole + vbowcap + vsterncap;
b = rho*v;

% Calculate the center of buoyancy, i.e. the centroid of the displaced
% fluid volume (assumming the vehicle is fully submerged) - measured
% from a NED world reference frame fixed at the rear end of the vehicle
cob_x = ((5*lsterncap/8)*vsterncap +...
    2*(lsterncap+ltunnel)*(vtunnel-vtunnelhole) +...
    (lsterncap+2*ltunnel+lhull/2)*vhull +...
    (lsterncap+2*ltunnel+(lhull-lsleeve)/3+lsleeve/2)*...
    (vsleeve+2*vwing+2*vmotormount+vgpsmount) +...
    2*(lsterncap+2*ltunnel+lhull+ltunnel)*(vtunnel-vtunnelhole) +...
    (lsterncap+2*ltunnel+lhull+2*ltunnel+3*lbowcap/8)*vbowcap)/v;
cob_y = 0; % symmetric about XZ plane
cob_z = (2*(dhull/2+lsleeve/2+thick+dmotor/2)*(cos(pi/3))*vmotormount +...
    2*(dhull/2+lsleeve/4)*(cos(pi/3))*vwing -...
    (dhull/2+3*thick/4)*vgpsmount)/v;

cob = [cob_x;cob_y;cob_z];
