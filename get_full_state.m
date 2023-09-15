function x_dot = get_full_state(x_in, u_in, param)
%%
ac1_state = x_in(1:12);

%  ctr1 = u_in(1:3);
% ctr1 = [0, 0, u_in(3)];%u_in(1:3);
ctr1 = [0, 0, 0];
thr1 = u_in(4)*20;


%extract vars from state input
XYZ1 = [ac1_state(1), ac1_state(2),ac1_state(3)]';
Eul1 = [ac1_state(7), ac1_state(8), ac1_state(9)]';

%extract constants
m = param(1);
moi_ac = [param(12), param(13), param(14)]';
d = param(17); k_line = param(18); 
coeff = zeros(6, 9);
coeff(1, 1) = param(19);coeff(1, 3) = param(20);coeff(2, 6) = param(21);
coeff(3, 1) = param(22);coeff(3, 3) = param(23);coeff(4, 8) = param(24);
coeff(4,8) = param(25);coeff(5, 3) = param(26);coeff(5, 5) = param(27);
coeff(6, 6) = param(28);coeff(6, 9) = param(29);
coeff_ctr = zeros(6, 3);
coeff_ctr(4, 1) = param(30);coeff_ctr(5, 2) = param(31);coeff_ctr(6, 3) = param(32);
yaw_moment_1 = param(33); pitch_c = param(34);roll_c_1 = param(35);

%get forces and moments before tension
[F_out1, M_tot1, dcm1] = get_ac_forces_moments(ac1_state, ctr1, param, coeff,coeff_ctr, yaw_moment_1, pitch_c, roll_c_1);
% [F_out1, M_tot1, dcm1] = get_ac_forces_moments(ac1_state, ctr1, param, coeff,coeff_ctr, yaw_moment_1, u_in(10)+pitch_c, u_in(9)+roll_c_1);
% [F_out2, M_tot2, dcm2] = get_ac_forces_moments(ac2_state, ctr2, param, coeff,coeff_ctr, yaw_moment_2, u_in(12)+pitch_c, u_in(11)+roll_c_2);
M_tot1 = M_tot1 + u_in(1:3)*20;

%tension
F_out1_inertial = inv(dcm1)*F_out1;
dist1 = sqrt(XYZ1(1)^2+XYZ1(2)^2+XYZ1(3)^2);
r1_norm = [XYZ1(1)/dist1, XYZ1(2)/dist1, XYZ1(3)/dist1]';
gen_d1 = d;
Force_elastic1_mag = (dist1-gen_d1)*k_line;
Force_elastic1 = -Force_elastic1_mag*r1_norm;
F_tot1_inertial = F_out1_inertial+Force_elastic1;
F_tot1 = dcm1*F_tot1_inertial;
% F_tot1 = dcm1*F_tot1_inertial + u_in(1:3);

%solve for output accelerations
u_dot1 = F_tot1(1)/m+thr1; v_dot1 = F_tot1(2)/m; w_dot1 = F_tot1(3)/m;
%p_dot1 = M_tot1(1)/moi_ac(1); q_dot1 = M_tot1(2)/moi_ac(2); r_dot1 = M_tot1(3)/moi_ac(3);

%extract vars from state input
u1 = ac1_state(4);v1 = ac1_state(5);w1 = ac1_state(6); 
p1 = ac1_state(10);q1 = ac1_state(11);r1 = ac1_state(12);

%6DOF
XYZ_dot1 = (inv(dcm1)*[u1, v1, w1]');

UVW_dot1 = [u_dot1, v_dot1, w_dot1]'+cross([u1, v1, w1]', [p1, q1, r1]');

J1 = [1, sin(Eul1(1))*tan(Eul1(2)), cos(Eul1(1))*tan(Eul1(2)); 0, cos(Eul1(1)), -sin(Eul1(1)); 0, sin(Eul1(1))/cos(Eul1(2)), cos(Eul1(1))/cos(Eul1(2))];
Eul_dot1 = (J1*[p1, q1, r1]');

Inertia = [moi_ac(1), 0, 0; 0, moi_ac(2), 0; 0, 0, moi_ac(3)];
M_eff1 = M_tot1-cross([p1, q1, r1]', Inertia*[p1, q1, r1]');
PQR_dot1 = (M_eff1'*inv(Inertia))';

%output derivatives
x_dot_ac1 = [XYZ_dot1', UVW_dot1', Eul_dot1', PQR_dot1'];
x_dot = [x_dot_ac1]';

%%

end