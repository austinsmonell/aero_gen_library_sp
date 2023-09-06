function [gamma1, gamma2, delta_gamma] = get_gamma(y)

%% Gamma
dcm1 = eul2rotm([y(9), y(8), y(7)])';
dcm2 = eul2rotm([y(21), y(20), y(19)])';

vel_dir_1 = inv(dcm1)*[normalize([y(4), y(5), y(6)], 'norm')]';
vel_dir_2 = inv(dcm2)*[normalize([y(16), y(17), y(18)], 'norm')]';

posi_dir_1 = [normalize([y(1), y(2), y(3)], 'norm')]';
posi_dir_2 = [normalize([y(13), y(14), y(15)], 'norm')]';

tang_vel_1 = vel_dir_1-dot(vel_dir_1, posi_dir_1)*posi_dir_1;
tang_vel_2 = vel_dir_2-dot(vel_dir_2, posi_dir_2)*posi_dir_2;

plane_ref_1 = normalize([0, 0, -1]' - dot([0, 0, -1]', posi_dir_1)*posi_dir_1, 'norm');
plane_ref_2 = normalize([0, 0, -1]' - dot([0, 0, -1]', posi_dir_2)*posi_dir_2, 'norm');


temp1 = real(acos(dot(tang_vel_1, plane_ref_1)/(sqrt(dot(tang_vel_1, tang_vel_1))*sqrt(dot(plane_ref_1, plane_ref_1)))));
temp2 = cross(plane_ref_1, tang_vel_1);
gamma1 = temp1*(temp2(1)/abs(temp2(1)));

temp1 = real(acos(dot(tang_vel_2, plane_ref_2)/(sqrt(dot(tang_vel_2, tang_vel_2))*sqrt(dot(plane_ref_2, plane_ref_2)))));
temp2 = cross(plane_ref_2, tang_vel_2);
gamma2 = temp1*(temp2(1)/abs(temp2(1)));

%% Delta Gamma
clear temp1;
clear temp2;
temp1(1) = dot(tang_vel_1, cross(posi_dir_1, plane_ref_1));
temp1(2) = dot(tang_vel_1, plane_ref_1);
temp2(1) = -dot(tang_vel_2, cross(posi_dir_2, plane_ref_2));
temp2(2) = dot(tang_vel_2, plane_ref_2);

temp3 = real(acos(dot(temp1, temp2)/(sqrt(dot(temp1, temp1))*sqrt(dot(temp2, temp2)))));
temp1(3) = 0;
temp2(3) = 0;
temp4 = cross(temp1, temp2);
if temp4(3) < 0
delta_gamma = -(temp3-pi)+pi;
else
    delta_gamma = temp3;
end


end