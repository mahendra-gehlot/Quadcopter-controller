% Simulation Constants
T1 = 3; % time for three parts
T2 = 2;
T3 = 3;
h = 1/1000;
t = (0:h:(T1+T2+T3));
L = length(t);

% quadcopter constants
m = 1.138; Ixx = 0.0077; Iyy = 0.0075; Izz = 0.0127;
g = 9.8 ; % gravity
% disturbance
rho_a = 1.2;
Vw = [3.2;0;0];
S_ref = 0.06;
d_ref = 0.28;

CL = 0.3;
CM = 0.3;
CN = 0.3;
Cl = 0.3;
Cn = 0.3;
Cm = 0.3;
lambda_rp = 8; lambda_z = 8; lambda_y = 8;