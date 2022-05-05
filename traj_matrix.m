function [A, C] = traj_matrix(T)

% for ninth order polynomial
A = [  1     T     T^2       T^3       T^4;
       5   6*T   7*T^2     8*T^3     9*T^4;
      20  30*T  42*T^2    56*T^3    72*T^4;
      60 120*T 210*T^2  56*6*T^3  72*7*T^4;
     120 360*T 840*T^2 56*30*T^3 72*42*T^4];

% for fifth  order polynomial
C = [1  T  T^2 ; 3  4*T  5*T^2; 6 12*T 20*T^2];

end