function Damp = generate_dampcoefficients(alpha_vec)

% Load polynomial - 5th order - fits for damping coefficients (per rad/s)
load 'AUV_DampCoeffFit.mat' Ckpfit Ckrfit Cmqfit Cnpfit Cnrfit;
% Evaluate for all values of alpha - positive values ONLY
Damp.Ckp = feval(Ckpfit,alpha_vec);
Damp.Ckr = feval(Ckrfit,alpha_vec);
Damp.Cmq = feval(Cmqfit,alpha_vec);
Damp.Cnp = feval(Cnpfit,alpha_vec);
Damp.Cnr = feval(Cnrfit,alpha_vec);
