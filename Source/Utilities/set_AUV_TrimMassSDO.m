% Reset trim masses and XY location parameters to starting values before
% running the optimization
xtrim_mass = 0; %#ok
ytrim_mass = 0; %#ok
xtrim = 0;
ytrim = 0;

% Configure simulation model for multiple test runs
set_param(gcs,'FastRestart','on','SimMechanicsOpenEditorOnUpdate','off');

% Run a single simulation to compute trim masses (for neutral buoyancy)
out = sim(gcs);

buoyancy = out.logsout.getElement('buoyancy').Values.Data(length(out.tout));
mass     = out.logsout.getElement('mass').Values.Data(length(out.tout));

mass_diff = buoyancy - mass;

% Set the trim mass values (arbitrarily set the ytrim mass to 1)
ytrim_mass = 1;
xtrim_mass = mass_diff - 1;

clear out buoyancy mass mass_diff

% Open the Response Optimization App and load previously saved session
load('AUV_MechanicsFullCAD_SDO_sdosession.mat')
sdotool(SDOSessionData)

