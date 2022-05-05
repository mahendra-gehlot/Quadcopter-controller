% Initialization of Variables & Matrices
phi = 0; theta = 0; psi = 0;
EA = [phi;theta;psi];
R = rotMat(EA(1), EA(2), EA(3)); 
B = [cos(theta)*cos(psi)   sin(psi)    0 ;
    -cos(theta)*sin(psi)   cos(psi)    0 ;
     sin(theta)                    0         1] ;
% Defining Initial Values and Terminal Values 
r(:,1) = [0;0;0]; psi(1) = 0;
V(:,1) = [0;0;0]; dEA(:,1) = [0;0;0];

v = R*V; w = B*dEA;
F_h = [m*g; 0];
X = [r;v;EA;w];
