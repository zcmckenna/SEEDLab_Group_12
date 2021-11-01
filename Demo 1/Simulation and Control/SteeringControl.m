%
% turning system parameters
%
K_phi=1;
sigma_phi=10;
%
%forward system parameters
%
K_rho=1;
sigma_rho=10;
%
%forward speed setpoint
rhodot_d = 0.1; %ft/s
%
%Initial Conditions
x_0 = 0;y_0=0;phi_0=0;

%line parameters
x_s = [0 3 6 10]
y_s = [0 0 3 3]
sim("SteeringControl_line")

animate