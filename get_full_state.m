function x_dot = get_full_state(x_in, u_in, param)
%%
ac1_state = x_in(1:12);
ac2_state = x_in(13:24);
gen_state = x_in(25:26);
%  ctr1 = u_in(1:3);
%  ctr2 = u_in(4:6);
% ctr1 = [0, 0, u_in(3)];%u_in(1:3);
% ctr2 = [0, 0, u_in(6)];%u_in(4:6);
ctr1 = [0, 0, 0];
ctr2 = [0, 0, 0];
thr1 = u_in(7)*20;
thr2 = u_in(8)*20;


%extract vars from state input
XYZ1 = [ac1_state(1), ac1_state(2),ac1_state(3)]';
XYZ2 = [ac2_state(1), ac2_state(2),ac2_state(3)]';
Eul1 = [ac1_state(7), ac1_state(8), ac1_state(9)]';
Eul2 = [ac2_state(7), ac2_state(8), ac2_state(9)]';

theta = gen_state(1);theta_dot = gen_state(2);

%extract constants
m = param(1);
moi_ac = [param(12), param(13), param(14)]';
r = param(17); d = param(18); k_line = param(19); 
k_damp = param(20); c = param(21); moi_gen = param(22);
coeff = zeros(6, 9);
coeff(1, 1) = param(23);coeff(1, 3) = param(24);coeff(2, 6) = param(25);
coeff(3, 1) = param(26);coeff(3, 3) = param(27);coeff(4, 8) = param(28);
coeff(4,8) = param(29);coeff(5, 3) = param(30);coeff(5, 5) = param(31);
coeff(6, 6) = param(32);coeff(6, 9) = param(33);
coeff_ctr = zeros(6, 3);
coeff_ctr(4, 1) = param(34);coeff_ctr(5, 2) = param(35);coeff_ctr(6, 3) = param(36);
yaw_moment_1 = param(37);yaw_moment_2 = param(38);
pitch_c = param(39);roll_c_1 = param(40);roll_c_2 = param(41);

%get forces and moments before tension
[F_out1, M_tot1, dcm1] = get_ac_forces_moments(ac1_state, ctr1, param, coeff,coeff_ctr, yaw_moment_1, pitch_c, roll_c_1);
[F_out2, M_tot2, dcm2] = get_ac_forces_moments(ac2_state, ctr2, param, coeff,coeff_ctr, yaw_moment_2, pitch_c, roll_c_2);
% [F_out1, M_tot1, dcm1] = get_ac_forces_moments(ac1_state, ctr1, param, coeff,coeff_ctr, yaw_moment_1, u_in(10)+pitch_c, u_in(9)+roll_c_1);
% [F_out2, M_tot2, dcm2] = get_ac_forces_moments(ac2_state, ctr2, param, coeff,coeff_ctr, yaw_moment_2, u_in(12)+pitch_c, u_in(11)+roll_c_2);
M_tot1 = M_tot1 + u_in(1:3)*20;
M_tot2 = M_tot2 + u_in(4:6)*20;

%tension
F_out1_inertial = inv(dcm1)*F_out1;
F_out2_inertial = inv(dcm2)*F_out2;
tmp = sqrt(XYZ1(1)^2+XYZ1(2)^2+XYZ1(3)^2);
r1_norm = [XYZ1(1)/tmp, XYZ1(2)/tmp, XYZ1(3)/tmp]';
tmp = sqrt(XYZ2(1)^2+XYZ2(2)^2+XYZ2(3)^2);
r2_norm = [XYZ2(1)/tmp, XYZ2(2)/tmp, XYZ2(3)/tmp]';
dist1 = norm(XYZ1);
dist2 = norm(XYZ2);
diff_gen = theta*r;
gen_d1 = d/2-diff_gen;
gen_d2 = d/2+diff_gen;
Force_elastic1_mag = (dist1-gen_d1)*k_line;
Force_elastic2_mag = (dist2-gen_d2)*k_line;
Force_elastic1 = -Force_elastic1_mag*r1_norm;
Force_elastic2 = -Force_elastic2_mag*r2_norm;
F_tot1_inertial = F_out1_inertial+Force_elastic1;
F_tot2_inertial = F_out2_inertial+Force_elastic2;
F_tot1 = dcm1*F_tot1_inertial;
F_tot2 = dcm2*F_tot2_inertial;
% F_tot1 = dcm1*F_tot1_inertial + u_in(1:3);
% F_tot2 = dcm2*F_tot2_inertial + u_in(4:6);

%generator
gen_moment_ac = (-Force_elastic1_mag+Force_elastic2_mag)*r;
gen_damping = c;%+theta*theta_dot*k_damp;
gen_damping_moment = -theta_dot*gen_damping;
gen_moment_tot = gen_moment_ac+gen_damping_moment;
theta_dot_dot = gen_moment_tot/moi_gen;

%solve for output accelerations
u_dot1 = F_tot1(1)/m+thr1; v_dot1 = F_tot1(2)/m; w_dot1 = F_tot1(3)/m;
%p_dot1 = M_tot1(1)/moi_ac(1); q_dot1 = M_tot1(2)/moi_ac(2); r_dot1 = M_tot1(3)/moi_ac(3);
u_dot2 = F_tot2(1)/m+thr2; v_dot2 = F_tot2(2)/m; w_dot2 = F_tot2(3)/m;
%p_dot2 = M_tot2(1)/moi_ac(1); q_dot2 = M_tot2(2)/moi_ac(2); r_dot2 = M_tot2(3)/moi_ac(3);

%extract vars from state input
u1 = ac1_state(4);v1 = ac1_state(5);w1 = ac1_state(6); 
p1 = ac1_state(10);q1 = ac1_state(11);r1 = ac1_state(12);
u2 = ac2_state(4);v2 = ac2_state(5);w2 = ac2_state(6); 
p2 = ac2_state(10);q2 = ac2_state(11);r2 = ac2_state(12);

%6DOF
XYZ_dot1 = (inv(dcm1)*[u1, v1, w1]');
XYZ_dot2 = (inv(dcm2)*[u2, v2, w2]');

UVW_dot1 = [u_dot1, v_dot1, w_dot1]'+cross([u1, v1, w1]', [p1, q1, r1]');
UVW_dot2 = [u_dot2, v_dot2, w_dot2]'+cross([u2, v2, w2]', [p2, q2, r2]');

J1 = [1, sin(Eul1(1))*tan(Eul1(2)), cos(Eul1(1))*tan(Eul1(2)); 0, cos(Eul1(1)), -sin(Eul1(1)); 0, sin(Eul1(1))/cos(Eul1(2)), cos(Eul1(1))/cos(Eul1(2))];
Eul_dot1 = (J1*[p1, q1, r1]');
J2 = [1, sin(Eul2(1))*tan(Eul2(2)), cos(Eul2(1))*tan(Eul2(2)); 0, cos(Eul2(1)), -sin(Eul2(1)); 0, sin(Eul2(1))/cos(Eul2(2)), cos(Eul2(1))/cos(Eul2(2))];
Eul_dot2 = (J2*[p2, q2, r2]');

Inertia = [moi_ac(1), 0, 0; 0, moi_ac(2), 0; 0, 0, moi_ac(3)];
M_eff1 = M_tot1-cross([p1, q1, r1]', Inertia*[p1, q1, r1]');
PQR_dot1 = (M_eff1'*inv(Inertia))';
M_eff2 = M_tot2-cross([p2, q2, r2]', Inertia*[p2, q2, r2]');
PQR_dot2 = (M_eff2'*inv(Inertia))';


%output derivatives
x_dot_ac1 = [XYZ_dot1', UVW_dot1', Eul_dot1', PQR_dot1'];
x_dot_ac2 = [XYZ_dot2', UVW_dot2', Eul_dot2', PQR_dot2'];
x_dot_gen = [theta_dot, theta_dot_dot];
x_dot = [x_dot_ac1, x_dot_ac2, x_dot_gen]';

%%

end