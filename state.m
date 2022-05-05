function dXdt = state(~, X, U, R, B, F, T)
constants;
vx = X(4); vy = X(5); vz = X(6);
wx = X(10); wy = X(11); wz = X(12);

f_h = U(1);
taux = U(2); tauy = U(3); tauz = U(4);

dr = R'*[vx; vy; vz];
dvx = wz*vy - wy*vz - g*R(1,3) + F(1)/m;
dvy = wx*vz - wz*vx - g*R(2,3) + F(2)/m;
dvz = wy*vx - wx*vy - g*R(3,3) + f_h/m + F(3)/m;

dEA = inv(B)*[wx; wy; wz];
dwx = (taux + T(1)- (Izz - Iyy)*wy*wz)/Ixx;
dwy = (tauy + T(2)- (Ixx - Izz)*wz*wx)/Iyy;
dwz = (tauz + T(3)- (Iyy - Ixx)*wx*wy)/Izz;

dXdt = [dr;dvx;dvy;dvz;dEA;dwx;dwy;dwz];

end