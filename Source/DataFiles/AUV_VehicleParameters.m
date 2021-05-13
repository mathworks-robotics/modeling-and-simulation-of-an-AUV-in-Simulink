%% AUV Vehicle Parameters
% This script loads up the default values for the parameterized components
% of the AUV model.

%% General Environment
Environment.gravity = 9.80665;                          % m/s^2
Environment.kinematicviscocity = 1.1923e-6;             % m/s^2 @15oC
Environment.sealevelpressure = 101325.0;                % N/m^2
Environment.seawaterdensity = 1027.3;                   % kg/m^3

% Define sea floor reference
Environment.seafloordepth = 50; %25;                         % m
Environment.seafloortileside = 20; %8;                       % m
Environment.seaflooropacity = 1;                        % 0...1

%% Vehicle Initial Conditions
Vehicle.InitCond.position = [0 0 5];                    % m
Vehicle.InitCond.velocity = [0 0 0];                    % m/s
Vehicle.InitCond.EulerXYZ = [0 0 0];                    % deg [roll,pitch,yaw]
Vehicle.InitCond.rotrates = [0 0 0];                    % rad/s

% Convert Euler angles to radians and compute rotation matrix
Vehicle.InitCond.EulerXYZrad = Vehicle.InitCond.EulerXYZ*pi/180;
Vehicle.InitCond.RotMZYX = eul2rotm(Vehicle.InitCond.EulerXYZrad,'ZYX');

%% Vehicle Body
Vehicle.Body.hulldiameter = 0.292;                      % m
Vehicle.Body.hullsectionlength = 0.74;                  % m
Vehicle.Body.tunnelsectionlength  = 0.14;               % m
Vehicle.Body.bowcaplength = 0.196;                      % m
Vehicle.Body.sterncaplength = 0.546;                    % m
Vehicle.Body.sterncapendradius = 0.0527;                % m
Vehicle.Body.wallthickness = 0.02;                      % m
Vehicle.Body.materialdensity = 1500;                    % kg/m^3

% Calculate the elliptical stern cap major axis
Vehicle.Body.sterncapmajoraxis = ...
    calc_ellipsemajoraxis(Vehicle.Body.hulldiameter/2,...
    Vehicle.Body.sterncapendradius,...
    Vehicle.Body.sterncaplength);

% Calculate vehicle total length
Vehicle.Body.totallength = Vehicle.Body.bowcaplength +...
    4*Vehicle.Body.tunnelsectionlength +...
    Vehicle.Body.hullsectionlength +...
    Vehicle.Body.sterncaplength;

% Calculate vehicle cross-sectional areas
Vehicle.Body.XZArea = ...
    pi*Vehicle.Body.hulldiameter*Vehicle.Body.bowcaplength/4 +...
    pi*Vehicle.Body.hulldiameter*Vehicle.Body.sterncaplength/4 +...
    Vehicle.Body.hulldiameter*(Vehicle.Body.hullsectionlength + ...
    4*Vehicle.Body.tunnelsectionlength);                % side
Vehicle.Body.XYArea = Vehicle.Body.XZArea;              % top
Vehicle.Body.YZArea = pi*Vehicle.Body.hulldiameter^2/4; % front

% Load vehicle hull inertial properties - for hull parametric model
% measured from the back end of the stern cap
load('AUV_HullInertialProperties.mat');
[Vehicle.Body.Hull.mass,Vehicle.Body.Hull.CoM,...
    Vehicle.Body.Hull.Inertia,Vehicle.Body.Hull.buoyancy,...
    Vehicle.Body.Hull.CoB] = extractInertialPropertiesHull(HullData);
clear HullData;

% Load vehicle hull inertial properties - for hull CAD model
% measured from the back end of the stern cap
load('AUV_HullCADInertialProperties.mat');
[Vehicle.Body.HullCAD.mass,Vehicle.Body.HullCAD.CoM,...
    Vehicle.Body.HullCAD.Inertia] = ...
    extractInertialPropertiesHullCAD(HullCADData);
clear HullCADData;

% Load full vehicle inertial properties - for full CAD model
% measured from the tip of the bow cap
load('AUV_FullCADTrimInertialProperties.mat');
[Vehicle.Body.Full.buoyancy,Vehicle.Body.Full.CoB,...
    Vehicle.Body.Full.mass,Vehicle.Body.Full.CoM,...
    Vehicle.Body.Full.Inertia] = ...
    extractInertialPropertiesFullCAD(FullCADTrimData);
clear FullCADTrimData;

% Load optimized CoM trim masses and frame displacements
load('AUV_FullCADTrimMasses.mat');
Vehicle.Body.Full.xtrim = trimData.xtrim;
Vehicle.Body.Full.ytrim = trimData.ytrim;
Vehicle.Body.Full.xtrim_mass = trimData.xtrim_mass;
Vehicle.Body.Full.ytrim_mass = trimData.ytrim_mass;
clear trimData

% Calculate total buoyancy and weigth forces
Vehicle.Body.Full.buoyancyforce = -Vehicle.Body.Full.buoyancy*...
    Environment.gravity;                % negative on Z
Vehicle.Body.Full.totalweight = Vehicle.Body.Full.mass*...
    Environment.gravity;                % positive on Z
% Calculate buoyancy force moment arm vector (vector to the vehicle CoM)
Vehicle.Body.Full.BArm = Vehicle.Body.Full.CoM - Vehicle.Body.Full.CoB;

%% Vehicle Main Propellers
Vehicle.Prop.motordiameter = 0.04255;                   % m
Vehicle.Prop.motormountlength = 0.14;                   % m
Vehicle.Prop.nozzlediameter = 0.11112;                  % m
Vehicle.Prop.propdiameter = 0.08763;                    % m
Vehicle.Prop.hubdiameter = 0.037592;                    % m
Vehicle.Prop.pitchangle = 0.375934;                     % rad
Vehicle.Prop.bladearea = 0.0004665901;                  % m^2
Vehicle.Prop.numblades = 7;

% Calculate propeller cross-sectional area
Vehicle.Prop.proparea = pi*Vehicle.Prop.propdiameter^2/4;
% Calculate propeller total expanded area
Vehicle.Prop.propEarea = Vehicle.Prop.numblades*Vehicle.Prop.bladearea;
% Calculate propeller expanded area ratio
Vehicle.Prop.propEAR = Vehicle.Prop.propEarea/Vehicle.Prop.proparea;
% Calculate propeller pitch/diameter ratio (P/D)
Vehicle.Prop.pitchratio = pi*tan(Vehicle.Prop.pitchangle);
% Calculate propeller full assembly cross-sectional area
Vehicle.Prop.YZArea = pi*Vehicle.Prop.nozzlediameter^2/4;

% Propellers moment arms (vector to the vehicle CoM)
Vehicle.Prop.SParmY = -0.22239;                         % m
Vehicle.Prop.SParmZ = -0.11594;                         % m
Vehicle.Prop.PParmY = 0.22239;                          % m
Vehicle.Prop.PParmZ = -0.11594;                         % m

% Generate propeller thrust and torque coefficients from efficiency curves
Vehicle.Prop.J_vec = 0:0.005:1.4; % eval range for the advance velocity ratio
Vehicle.Prop.Coeff = generate_propcoefficients(Vehicle.Prop.J_vec,...
    Vehicle.Prop.pitchratio,Vehicle.Prop.propEAR,Vehicle.Prop.numblades);

% Load reduced cubic polynomial fit for kt and kq
load('AUV_PropPolyFit.mat');
Vehicle.Prop.KtPoly3 = coeffvalues(KtPolyFit);
Vehicle.Prop.KqPoly3 = coeffvalues(KqPolyFit);
clear KtPolyFit KqPolyFit

%% Vehicle Tunnel Thrusters
Vehicle.Thruster.tunneldiameter = 0.08;                 % m
Vehicle.Thruster.propdiameter = 0.07035;                % m
Vehicle.Thruster.hubdiameter = 0.030029;                % m
Vehicle.Thruster.pitchangle = 0.4117143;                % rad
Vehicle.Thruster.bladearea = 0.0002995275;              % m^2
Vehicle.Thruster.numblades = 7;

% Calculate propeller cross-sectional area
Vehicle.Thruster.proparea = pi*Vehicle.Thruster.propdiameter^2/4;
% Calculate propeller total expanded area
Vehicle.Thruster.propEarea = Vehicle.Thruster.numblades*Vehicle.Thruster.bladearea;
% Calculate propeller expanded area ratio
Vehicle.Thruster.propEAR = Vehicle.Thruster.propEarea/Vehicle.Thruster.proparea;
% Calculate propeller pitch/diameter ratio (P/D)
Vehicle.Thruster.pitchratio = pi*tan(Vehicle.Thruster.pitchangle);

% Calculate thruster X-axis locations (wrt the tip of the bow cap)
Vehicle.Thruster.BHxpos = Vehicle.Body.bowcaplength +...
    Vehicle.Body.tunnelsectionlength/2;
Vehicle.Thruster.BVxpos = Vehicle.Body.bowcaplength +...
    3*Vehicle.Body.tunnelsectionlength/2;
Vehicle.Thruster.SVxpos = Vehicle.Body.bowcaplength +...
    Vehicle.Body.hullsectionlength+5*Vehicle.Body.tunnelsectionlength/2;
Vehicle.Thruster.SHxpos = Vehicle.Body.bowcaplength +...
    Vehicle.Body.hullsectionlength+7*Vehicle.Body.tunnelsectionlength/2;

% Calculate thruster moment arms (vector to the vehicle CoM)
Vehicle.Thruster.BHarmX = Vehicle.Body.Full.CoM(1)+Vehicle.Thruster.BHxpos;
Vehicle.Thruster.BHarmZ = Vehicle.Body.Full.CoM(3);
Vehicle.Thruster.BVarmX = Vehicle.Body.Full.CoM(1)+Vehicle.Thruster.BVxpos;
Vehicle.Thruster.SVarmX = Vehicle.Thruster.SVxpos+Vehicle.Body.Full.CoM(1);
Vehicle.Thruster.SHarmX = Vehicle.Thruster.SHxpos+Vehicle.Body.Full.CoM(1);
Vehicle.Thruster.SHarmZ = Vehicle.Body.Full.CoM(3);

% Generate thruster thrust and torque coefficients from efficiency curves
Vehicle.Thruster.J_vec = 0:0.005:1.4; % eval range for the advance velocity ratio
Vehicle.Thruster.Coeff = generate_propcoefficients(Vehicle.Thruster.J_vec,...
    Vehicle.Thruster.pitchratio,Vehicle.Thruster.propEAR,...
    Vehicle.Thruster.numblades);

% Load reduced cubic polynomial fit for kt and kq


%% Vehicle Drag and Lift Coefficients
% Define the search ranges for the incidence angles
Vehicle.IncRange.alpha_vec = -15:3:30;                  % deg
Vehicle.IncRange.beta_vec =  -15:3:15;                  % deg

% Generate drag and lift coefficient arrays
Vehicle.Coeff = generate_hydrocoefficients(Vehicle.IncRange.alpha_vec,...
    Vehicle.IncRange.beta_vec);

%% Vehicle Damping Coefficients (per rad/sec)
% Extract ONLY positive range from alpha vector
Vehicle.IncRange.alpha_vec_damp =...
    Vehicle.IncRange.alpha_vec(Vehicle.IncRange.alpha_vec >= 0);
% Generate damping coefficient vectors
Vehicle.Damp = generate_dampcoefficients(Vehicle.IncRange.alpha_vec_damp);

%% Vehicle Added Mass Matrix
% Calculate added mass matrix coefficients
Vehicle.AddedMass = calc_addedmassMatrix(Vehicle.Body.hulldiameter/2,...
    Vehicle.Body.bowcaplength,Vehicle.Body.hullsectionlength +...
    4*Vehicle.Body.tunnelsectionlength,Vehicle.Body.sterncaplength,...
    Vehicle.Body.sterncapmajoraxis,Environment.seawaterdensity);

%% Vehicle Battery
Vehicle.Battery.mass = 3.3;                                 % kg
Vehicle.Battery.powerdensity = 300;                         % W/kg
Vehicle.Battery.energydensity = 120;                        % W*h/kg
Vehicle.Battery.numbatteries = 4;

Vehicle.Battery.voltage = 48;                               % V
Vehicle.Battery.resistance = 1e-2;                          % ohm
Vehicle.Battery.capacity = Vehicle.Battery.mass*...
    Vehicle.Battery.energydensity/Vehicle.Battery.voltage;  % A*h
Vehicle.Battery.v1atAH1 = 0.9*Vehicle.Battery.voltage;      % V
Vehicle.Battery.AH1atV1 = 0.5*Vehicle.Battery.capacity;     % A*h
Vehicle.Battery.R1 = 0.025;                                 % ohm
Vehicle.Battery.tau1 = 50;                                  % sec
Vehicle.Battery.initcapacity = Vehicle.Battery.capacity;    % A*h

%% Vehicle IMU
% Vector to the vehicle CoM (measured from CAD)
Vehicle.IMU.OffsetfromCoMX = -0.44657;                  % m
Vehicle.IMU.OffsetfromCoMY = 0;                         % m
Vehicle.IMU.OffsetfromCoMZ = 0.02905;                   % m

%% Define Visual Properties for Mechanics Explorer
Vehicle.Body.color1 = [255,237, 33]/255;                % RGB-YELLOW
Vehicle.Body.color2 = [255, 50,  0]/255;                % RGB-RED
Vehicle.Body.color3 = [102,204,  0]/255;                % RGB-GREEN
Vehicle.Body.color4 = [  2,131,208]/255;                % RGB-BLUE
Vehicle.Body.color5 = [ 82, 82, 82]/255;                % RGB-DARK GRAY
Vehicle.Body.opacity = 1;                               % 0...1
Vehicle.Body.noopacity = 0;                             % 0...1
Vehicle.Body.buoyopacity = 0;                           % 0...1
Vehicle.Body.markeropacity = 1;                         % 0...1
Vehicle.Body.shellopacity = 0.25;                       % 0...1


