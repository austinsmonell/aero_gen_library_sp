function R = eul2rotmat(eul)
%EUL2ROTM Convert Euler angles to rotation matrix

%R = zeros(3,3,size(eul,1));
R = zeros(3);
R(1, 1) = cos(eul(2))*cos(eul(1));
R(1,2) = sin(eul(3))*sin(eul(2))*cos(eul(1)) - cos(eul(3))*sin(eul(1));
R(1,3) = cos(eul(3))*sin(eul(2))*cos(eul(1)) + sin(eul(3))*sin(eul(1));
R(2,1) = cos(eul(2))*sin(eul(1));
R(2,2) = sin(eul(3))*sin(eul(2))*sin(eul(1)) + cos(eul(3))*cos(eul(1));
R(2,3) = cos(eul(3))*sin(eul(2))*sin(eul(1)) - sin(eul(3))*cos(eul(1));
R(3,1) = -sin(eul(2));
R(3,2) = sin(eul(3))*cos(eul(2));
R(3,3) = cos(eul(3))*cos(eul(2));
 
end

