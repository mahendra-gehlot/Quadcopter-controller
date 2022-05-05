clear;
close all;
clc;
constants;

%%  perallocating matrix
initialize;              % initilizing values of all the variables

%% Generate Trajectory
generateTraj;

%% Main Loop
for i = 1:L

% Rotation matrix from earth-fixed frame to body fixed frame.
R = rotMat(phi(i),theta(i),psi(i));

% Tranformation matrix to relate Euler angle rates to the angular velocity vector in a body-fixed frame
B = [cos(theta(i))*cos(psi(i))   sin(psi(i))    0 ;
    -cos(theta(i))*sin(psi(i))   cos(psi(i))    0 ;
     sin(theta(i))                    0         1] ;

%% Error dynamics for roll and pitch moment
error_rg(:,i) = r(:,i) - rd(:,i);
if i == 1
   Irp(:,i) = h*error_rg(:,i)/2;
elseif i == 2
   Irp(:,i) = h*( error_rg(:,i-1) + error_rg(:,i) )/2;
else
   Irp(:,i) = ( error_rg(:,i) + 2/h*Irp(:,i-1) + error_rg(:,i-1) )*h/2;
end

gamma_rp(:,i) = m*R*( -rd_dddd(:,i) - 5*lambda_rp*rd_ddd(:,i) - 10*lambda_rp^2*rd_dd(:,i) ...
                + 10*lambda_rp^3*(V(:,i)-rd_d(:,i)) + 5*lambda_rp^4*error_rg(:,i) + lambda_rp^5*Irp(:,i) );

%% calculating nominal force
F_h(:,i+1) = RK4(@(t, x) thrust(t, x, w(:,i), R, gamma_rp(3,i)), t(i), F_h(:,i), h);

f_hat = F_h(1,i+1);
df_hat = F_h(2,i+1);

% Input torque x and y
torque(1,i) = (Ixx - Iyy + Izz)*w(2,i)*w(3,i) - 5*lambda_rp*Ixx*w(1,i) ...
              - Ixx/f_hat*(2*w(1,i)*df_hat + 10*lambda_rp^2*m*g*R(2,3) - gamma_rp(2,i));

torque(2,i) = (Ixx - Iyy - Izz)*w(3,i)*w(1,i) - 5*lambda_rp*Iyy*w(2,i) ...
              + Iyy/f_hat*(-2*w(2,i)*df_hat + 10*lambda_rp^2*m*g*R(1,3) - gamma_rp(1,i));

%% Error dynamics for terms thrust term
if i == 1
   Iz(1,i) = h*error_rg(3,i)/2;
elseif i == 2
   Iz(1,i) = h*( error_rg(3,i-1) + error_rg(3,i) )/2;
else
   Iz(1,i) = ( error_rg(3,i-1) + 2/h*Iz(1,i-1) + error_rg(3,i) )*h/2;
end

% Input thrust
f(i) = m/R(3,3)*( g + rd_dd(3,i) - 3*lambda_z*(V(3,i)-rd_d(3,i)) ...
                 -3*lambda_z^2*error_rg(3,i) - lambda_z^3*Iz(1,i) ) ; 

%% Error dynamics for terms yaw term
error_y(1,i) = psi(1,i) - yawd(1,i);
if i == 1
   Iy(:,i) = h*error_y(1,i)/2;
elseif i == 2
   Iy(:,i) = h*( error_y(1,i-1) + error_y(1,i) )/2;
else
   Iy(:,i) = ( error_y(1,i-1) + 2/h*Iy(:,i-1) + error_y(1,i) )*h/2;
end

dwx = (torque(1,i) - (Izz - Iyy)*w(2,i)*w(3,i))/Ixx ;
dwy = (torque(2,i) - (Ixx - Izz)*w(3,i)*w(1,i))/Iyy ;
ddphi = 1/cos(theta(i))*( ( sin(theta((i)))*dEA(1,i) - dEA(3,i) )*dEA(2,i) + cos(psi(i))*dwx - sin(psi(i))*dwy);

% Input torque z
torque(3,i) = (Iyy-Ixx)*w(1,i)*w(2,i) + Izz*(sin(theta(i))*ddphi + ...
               cos(theta(i))*dEA(2,i)*dEA(3,i) + yawd_dd(1,i) - 3*lambda_y*(dEA(3,i) - yawd_d(1,i)) ...
               - 3*lambda_y^2*error_y(1,i) - lambda_y^3*Iy(1,i));

%% Disturbances
Fd = -R*0.5*rho_a*norm(V(:,i)-Vw)^2*S_ref*[CL;CM;CN];
Td = -R*0.5*rho_a*norm(V(:,i)-Vw)^2*S_ref*d_ref*[Cl;Cm;Cn];

%% Updating the velocity and position using Kinematic and Dynamics equation
U(:,i) = [f(i); torque(:,i)];
X(:,i+1) = RK4(@(t, x) state(t, x, U(:,i), R, B, Fd, Td), t(i), X(:,i), h);
r(:,i+1) = X(1:3,i+1);
V(:,i+1) = R'*X(4:6,i+1);
EA(:,i+1) = X(7:9,i+1);
w(:,i+1) = X(10:12,i+1);
dEA(:,i+1) = inv(B)*w(:,i+1);
phi(i+1) = EA(1,i+1); theta(i+1) = EA(2,i+1); psi(i+1) = EA(3,i+1);


% Imposed conditions -pi/2 < phi; theta < pi/2;
while (abs(phi(i+1))>=pi/2)
    n = abs(floor(phi(i+1)/pi));
    if (abs(phi(i+1))<pi)
        n = 1;
    end
    phi(i+1) = (abs(phi(i+1)) - n*pi)*sign(phi(i+1));
end
while (abs(theta(i+1))>pi/2)
    n = abs(floor(theta(i+1)/pi));
    if (abs(theta(i+1))<pi)
        n = 1;
    end
    theta(i+1) = (abs(theta(i+1)) - n*pi)*sign(theta(i+1));
end

end 

figure(1); 
plot(t, error_rg(1,:));
hold on; grid on;
plot(t, error_rg(2,:));
plot(t, error_rg(3,:));
legend('x(m)','y(m)','z(m)')
xlabel('Time (sec)');
ylabel('Trajectory Tracking Error (m)');
title("Trajectory Tracking Error vs Time $t$", Interpreter="latex")

figure(2); 
plot(t, EA(1,1:end-1));
hold on; grid on;
plot(t, EA(2,1:end-1));
plot(t, EA(3,1:end-1));
legend('roll(rad)','pitch(rad)','yaw(rad)')
xlabel('Time (sec)');
ylabel('Euler angles (rad)');
title("Euler angles vs Time $t$", Interpreter="latex")

figure(3); 
plot(t, V(1,1:end-1));
hold on; grid on;
plot(t, V(2,1:end-1));
plot(t, V(3,1:end-1));
legend('Vx(m/s)','Vy(m/s)','Vz(m/s)')
xlabel('Time (sec)');
ylabel('Linear Velocities in earth frame (m/s)');
title("Linear Velocities in earth frame $m/s$ vs Time $t$", Interpreter="latex")

figure(4); 
plot(t, dEA(1,1:end-1));
hold on; grid on;
plot(t, dEA(2,1:end-1));
plot(t, dEA(3,1:end-1));
legend('$\dot{\phi} (rad/s)$','$\dot{\theta} (rad/s)$','$\dot{\psi} (rad/s)$', Interpreter="latex")
xlabel('Time (sec)');
ylabel(' Euler angle rate (rad/s)');
title("Euler angle rate (rad/s) vs Time $t$", Interpreter="latex")

figure(5); 
plot3(r(1,:),r(2,:),r(3,:),'b');
hold on ;grid on;
plot3(rd(1,:),rd(2,:),rd(3,:), '-.r');
axis([-0.5 0.5 -1 2 -1 2]);

legend('Simulated','Desired')
xlabel(' x (m)');
ylabel(' y (m)');
zlabel(' z (m)');
title("Desired and simulated Trajectories ", Interpreter="latex")