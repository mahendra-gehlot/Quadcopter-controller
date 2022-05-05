% *************************************************************************

% Generate Trajectory

% *************************************************************************
%% Reference trajectory
[A1, C1] = traj_matrix(T1); 
[A2, C2] = traj_matrix(T2);
[A3, C3] = traj_matrix(T3);

%% parts of triangular trajectory
traj_di1 = [r(:,1);psi(:,1)]; % All other derivative values at both initial and final taken as zero
traj_df1 = traj_di1 + [0;0;1;0];
traj_di2 = traj_df1;
traj_df2 = traj_di2 + [0;1;0;0];
traj_di3 = traj_df2;
traj_df3 = traj_di1;

Coeff_Pos1 = inv(A1)*[(traj_df1(1:end-1)-traj_di1(1:end-1))'/T1^5; zeros(4,3)];
Coeff_Yaw1 = inv(C1)*[(traj_df1(4)-traj_di1(4))/T1^3;0;0];

Coeff_Pos2 = inv(A2)*[(traj_df2(1:end-1)-traj_di2(1:end-1))'/T2^5;  zeros(4,3)];
Coeff_Yaw2 = inv(C2)*[(traj_df2(4)-traj_di2(4))/T2^3;0;0];

Coeff_Pos3 = inv(A3)*[(traj_df3(1:end-1)-traj_di3(1:end-1))'/T3^5; zeros(4,3)];
Coeff_Yaw3 = inv(C3)*[(traj_df3(4)-traj_di3(4))/T3^3;0;0];

%% trajectory leader
for i = 1:L

if i <= T1/(T1+T2+T3)*L
[rd(:,i), rd_d(:,i) , rd_dd(:,i) , rd_ddd(:,i), rd_dddd(:,i), yawd(1,i),  yawd_d(1,i), yawd_dd(1,i)] = desired_trajectory_generation(t(i),Coeff_Pos1,Coeff_Yaw1,traj_di1);

elseif i <= (T1+T2)/(T1+T2+T3)*L 
[rd(:,i), rd_d(:,i) , rd_dd(:,i) , rd_ddd(:,i), rd_dddd(:,i), yawd(1,i),  yawd_d(1,i), yawd_dd(1,i)] = desired_trajectory_generation(t(i-T1/h),Coeff_Pos2,Coeff_Yaw2,traj_di2);

else
[rd(:,i), rd_d(:,i) , rd_dd(:,i) , rd_ddd(:,i), rd_dddd(:,i), yawd(1,i),  yawd_d(1,i), yawd_dd(1,i)] = desired_trajectory_generation(t(i-(T1+T2)/h),Coeff_Pos3,Coeff_Yaw3,traj_di3);

end
end