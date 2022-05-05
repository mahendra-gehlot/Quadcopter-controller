function R = rotMat(phi,theta,psi)

% Rotation matrix 
R(1,1) = cos(theta)*cos(psi);
R(1,2) = cos(phi)*sin(psi) + sin(phi)*sin(theta)*cos(psi);
R(1,3) = sin(phi)*sin(psi) - cos(phi)*sin(theta)*cos(psi);
R(2,1) = -cos(theta)*sin(psi);
R(2,2) = cos(phi)*cos(psi) - sin(phi)*sin(theta)*sin(psi);
R(2,3) = sin(phi)*cos(psi) + cos(phi)*sin(theta)*sin(psi);
R(3,1) = sin(theta);
R(3,2) = -sin(phi)*cos(theta);
R(3,3) = cos(phi)*cos(theta);

end