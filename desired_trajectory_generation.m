function [rd,rd_d,rd_dd,rd_ddd,rd_dddd,yd,yd_d,yd_dd] = desired_trajectory_generation(t, B_Coeff,B1,traj_di)
rd = (t^5*[1 t t^2 t^3 t^4]*B_Coeff)' + traj_di(1:3);
rd_d = (t^4*[5 6*t 7*t^2 8*t^3 9*t^4]*B_Coeff)';
rd_dd = (t^3*[5*4 6*5*t 7*6*t^2 8*7*t^3 9*8*t^4]*B_Coeff)';
rd_ddd = (t^2*[5*4*3 6*5*4*t 7*6*5*t^2 8*7*6*t^3 9*8*7*t^4]*B_Coeff)';
rd_dddd = (t*[5*4*3*2  6*5*4*3*t  7*6*5*4*t^2  8*7*6*5*t^3  9*8*7*6*t^4]*B_Coeff)';

yd =    (t^3*[1 t t^2]*B1)' + traj_di(4);
yd_d =  (t^2*[3 4*t 5*t^2]*B1)';
yd_dd = (t^1*[3*2 4*3*t 5*4*t^2]*B1)';
end