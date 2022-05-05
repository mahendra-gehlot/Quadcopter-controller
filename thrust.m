function dFdt = thrust(~, F, W, R, C)
constants;
f = F(1);
dfdt = F(2);

ddfdt = -5*lambda_z*dfdt - (10*lambda_z^2 - W(1)^2 - W(2)^2)*f + 10*lambda_z^2*m*g*R(3,3) - C;

dFdt = [dfdt; ddfdt];
end