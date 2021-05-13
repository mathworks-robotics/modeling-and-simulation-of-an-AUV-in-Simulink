function Ma = calc_addedmassMatrix(rh,lb,lm,ls,lse,rho)
% CALC_ADDEDMASSMATRIX calculate added mass matrix coefficients
%   Ma = calc_addedmassMatrix(r,lb,lm,ls,rho) applies slender
%   body theory to calculate the added mass matrix coefficients for
%   the vehicle body.
%
%   Inputs:
%   rh  : radius of the hull
%   lb  : major axis length of the elliptical bow cap section of the hull
%   lm  : length of the middle section of the hull
%   ls  : actual length of the elliptical stern cap section of the hull
%   lse : major axis length of the elliptical stern cap section of the hull
%   rho : density of the fluid
%   Output:
%   Ma  : added mass matrix
%
%   The function considers the hull of the vehicle by itself, so it
%   assumes rotational symmetry around the x-axis.
%   Given that assumption, the added mass matrix is reduced to:
%
%                        | m11  0   0   0   0   0  |
%                        |  0  m22  0   0   0  m26 |
%                   Ma = |  0   0  m33  0  m35  0  |
%                        |  0   0   0   0   0   0  |
%                        |  0   0  m53  0  m55  0  |
%                        |  0  m62  0   0   0  m66 |
%
%   where m22 = m33 and m55 = m66

% In order to calculate the total added mass coefficients for the
% hull using circular cylinder sections as the base shape, we will
% divide the hull lenght into three segments and perform an integration
% of the 2-dimensional coefficients on each of the separate segments
% and add them together

% bow segment - elliptical cap - radius is r(x)
fun1_b = @(x) pi*rho*rh^2*(1-x.^2/lb^2);
% stern segment - elliptical cap - radius is r(x)
fun1_s = @(x) pi*rho*rh^2*(1-x.^2/lse^2);

m22 = integral(fun1_b,0,lb)+pi*rho*rh^2*lm+integral(fun1_s,0,ls);
m33 = m22;

% bow segment - elliptical cap - radius is r(x)
fun2_b = @(x) pi*rho*rh^2*(1-x.^2/lb^2).*(x+lm/2).^2;%----------fixed the distance from center part
% stern segment - elliptical cap - radius is r(x)
fun2_s = @(x) pi*rho*rh^2*(1-x.^2/lse^2).*(-lm/2+x).^2;%----------fixed the distance from center part

%%fixed the piece about mid section-------------integral(pi*rho*rh^2*x^2,x=-lm/2,x=lm/2)
m55 = integral(fun2_b,0,lb)+pi*rho*rh^2*((lm/2)^3/3-(-lm/2)^3/3)+integral(fun2_s,-ls,0);
m66 = m55;

% bow segment - elliptical cap - radius is r(x)
fun3_b = @(x) pi*rho*rh^2*(1-x.^2/lb^2).*(x+lm/2);%----------fixed the distance from center part
% stern segment - elliptical cap - radius is r(x)
fun3_s = @(x) pi*rho*rh^2*(1-x.^2/lse^2).*(-lm/2+x);%----------fixed the distance from center part

m26 = integral(fun3_b,0,lb)+pi*rho*rh^2*rh*((lm/2)^2/2-(-lm/2)^2/2)+integral(fun3_s,-ls,0);%-------------fixed the range for integration and the middle part
m62 = m26; %-------------fixed signs; no need for negative
m35 = m26; %-------------fixed signs; no need for negative
m53 = m35; %-------------fixed signs; no need for negative

% Slender body theory calculates the added mass matrix coefficients for
% the sections perpendicular to the x-axis.
% To calculate the axial added mass in the x-direction we will use Lamb's
% k-factors and approximate the vehicle hull by a prolate spheroid

% In order to approximate the vehicle hull with a spheroid we will use
% the equivalent ellipsoid method, where the diameter and volume of the
% spheroid is equal to the diameter and volume of the actual hull
Vhull = 4/3*pi*lb*rh^2/2 + lm*pi*rh^2 + 4/3*pi*lse*rh^2/2;
r1 = 3*Vhull/(4*pi*rh^2);

% Eccentricity of the elliptical section where r1 is the half-length of
% the spheroid and rh is the radius
e = sqrt(1 - (rh/r1)^2);
% Constant describing the relative proportion of the spheroid
a0 = 2*(1-e^2)/e^2*(1/2*log((1+e)/(1-e))-e);
% Lamb's k-factor
k1 = a0/(2-a0);
% Axial added mass coefficient
m11 = k1*4/3*pi*rho*r1*rh^2;

% Constructing the complete added mass matrix
Ma = [m11  0   0   0   0   0   ;
       0  m22  0   0   0  m26  ;
       0   0  m33  0  m35  0   ;
       0   0   0   0   0   0   ;
       0   0  m53  0  m55  0   ;
       0  m62  0   0   0  m66 ];


