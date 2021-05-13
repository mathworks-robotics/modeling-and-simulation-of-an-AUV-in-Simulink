%% Define the Underwater Channel
propSpeed = 1520;
channelDepth = 200;
OperatingFrequency = 37.5e3; 

isopaths = phased.IsoSpeedUnderwaterPaths('ChannelDepth',channelDepth,...
  'NumPathsSource','Property','NumPaths',1,'PropagationSpeed',propSpeed);

channel = phased.MultipathChannel('OperatingFrequency',OperatingFrequency);


%% Define the Acoustic Beacon and Passive Array
prf = 1;
pulseWidth = 10e-3;
pulseBandwidth = 1/pulseWidth;
fs = 2*pulseBandwidth;
wav = phased.RectangularWaveform('PRF',prf,'PulseWidth',pulseWidth,...
  'SampleRate',fs);
channel.SampleRate = fs;

projector = phased.IsotropicProjector('VoltageResponse',120);
projRadiator = phased.Radiator('Sensor',projector,...
  'PropagationSpeed',propSpeed,'OperatingFrequency',OperatingFrequency);
beaconPlat = phased.Platform('InitialPosition',[5000; 2000; -199],...
  'Velocity',[0; 0.1; 0]);

hydrophone = phased.IsotropicHydrophone('VoltageSensitivity',-150);
array = phased.ULA('Element',hydrophone,...
  'NumElements',5,'ElementSpacing',propSpeed/OperatingFrequency/2,...
  'ArrayAxis','y');
arrayCollector = phased.Collector('Sensor',array,...
  'PropagationSpeed',propSpeed,'OperatingFrequency',OperatingFrequency);
arrayPlat = phased.Platform('InitialPosition',[0; 0; -10],...
  'Velocity',[0; 0; 0]);

rx = phased.ReceiverPreamp(...
    'Gain',20,...
    'NoiseFigure',10,...
    'SampleRate',fs,...
    'SeedSource','Property',...
    'Seed',2007);




%% Simulate the Passive Sonar System

x = [[.707+0.707i 2 3 6 4]'; 1 ;zeros(50+94,1);zeros(50,1) ];

numTransmits = 5;
rxsig = zeros(size(x,1),5,numTransmits);
for i = 1:numTransmits

  % Update array and acoustic beacon positions
  [pos_tx,vel_tx] = beaconPlat(1/prf);
  [pos_rx,vel_rx] = arrayPlat(1/prf);

  % Compute paths between the acoustic beacon and array
  [paths,dop,aloss,rcvang,srcang] = ...
      isopaths(pos_tx,pos_rx,vel_tx,vel_rx,1/prf);

  % Propagate the acoustic beacon waveform
  tsig = projRadiator(x,srcang);
  rsig = channel(tsig,paths,dop,aloss);
  
  % Collect the propagated signal
  rsig = arrayCollector(rsig,rcvang);
  
  % Store the received pulses
  rxsig(:,:,i) = abs(rx(rsig));%original
  %rxsig(:,:,i) = abs(rsig);
  
end
  y = [rxsig(110,end),rxsig(111,end), rxsig(112,end), rxsig(113,end), rxsig(114,end), rxsig(115,end)]';
  y = y - mean(rxsig(:,end));
  y = y /(rxsig(110,end) - mean(rxsig(:,end)))
%   y = abs(floor(y(1:5)))
  



