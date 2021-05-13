function Coeff = generate_hydrocoefficients(alpha_vec,beta_vec)

% Define hydrodynamic drag and lift characteristic polynomial coefficients
%             Lift       Pitch Moment       Drag
PCoeff = [-9.0250e-03     0.1320e-02     1.8405e-01
           4.0700e-03    -2.2260e-03    -6.4000e-04
           3.0940e-06    -1.8590e-05    -5.5200e-04
           1.5640e-06     6.0010e-07    -5.6550e-06
          -1.3860e-07     1.8280e-07     7.4150e-07
           2.5450e-09    -9.7330e-09    -9.1000e-09
          -1.1890e-11     1.7100e-10     2.3470e-11
           2.5640e-04    -5.2330e-04    -1.3247e-03
           8.5010e-05     6.7950e-05    -1.1772e-03
          -1.1560e-05    -1.9930e-05     2.1430e-04
           3.4160e-07     1.3410e-06    -1.0497e-05
          -4.8620e-05     6.0610e-05     3.2375e-04];

% Generate square array pairs for all value combinations of alpha and beta
% and reshape them into column vectors
[AL,BE]=meshgrid(alpha_vec,beta_vec);
AL=AL(:);
BE=BE(:);

% Evaluate polynomials for all values of alpha and beta
PEval = [ones(length(AL),1),AL,AL.^2,AL.^3,AL.^4,AL.^5,AL.^6,...
    abs(BE),BE.^2,abs(BE.^3),BE.^4,AL.*abs(BE)]*PCoeff;
% Select the appropriate column and reshape the arrays
Coeff.CZ  = -reshape(PEval(:,1),length(beta_vec),length(alpha_vec));
Coeff.CMM =  reshape(PEval(:,2),length(beta_vec),length(alpha_vec));
Coeff.CX  = -reshape(PEval(:,3),length(beta_vec),length(alpha_vec));

% Mirror lift polynomial for lateral force coefficients
PEval = [ones(length(BE),1),BE,BE.^2,BE.^3,BE.^4,BE.^5,BE.^6,...
    abs(AL),AL.^2,abs(AL.^3),AL.^4,BE.*abs(AL)]*PCoeff(:,1);
Coeff.CY = -reshape(PEval(:,1),length(beta_vec),length(alpha_vec))*0.5;

% Roll moment coefficient (linear on beta)
Coeff.CMK = -0.0075*beta_vec'*ones(1,length(alpha_vec));

% Load polynomial fit for yaw moment coefficients
load 'AUV_CMNCoeffFit.mat' CMNfit;
% Evaluate for all values of alpha and beta
Coeff.CMN = feval(CMNfit,alpha_vec',beta_vec)'; 
