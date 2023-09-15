function [F_out, M_out, dcm] = get_ac_forces_moments(state, ctr, param, coeff,coeff_ctr, yaw_moment_coeff, pitch_c, roll_c)
%extract vars from state input
x = state(1);y = state(2);z = state(3);
u = state(4);v = state(5);w = state(6);
phi = state(7);theta = state(8);psi = state(9);
p = state(10);q = state(11);r = state(12);
%extract vars from control input
da = ctr(1);de = ctr(2);dr = ctr(3);

%extract constants
m = param(1);rho = param(2);
S = param(3);chord = param(4); span = param(5);
gravity = [param(6), param(7),param(8)];
wind = [param(9), param(10), param(11)]';
roll_k = param(15);pitch_k = param(16);

%intermediate variables
dcm = eul2rotmat([psi, theta, phi])';
wind_body = dcm*wind;
v_a_vec = [u, v, w]'-wind_body;
v_a = sqrt(v_a_vec(1)^2+v_a_vec(2)^2+v_a_vec(3)^2);
q_bar = 0.5*rho*v_a^2;
alpha = atan2(v_a_vec(3), v_a_vec(1));
beta = asin(v_a_vec(2)/v_a);

%non-dimensional aero coefficients
CD0 = coeff(1, 1);CDA = coeff(1, 3);
CYB = coeff(2, 6);
CL0 = coeff(3, 1);CLA = coeff(3, 3);
ClP = coeff(4, 8);ClDa = coeff_ctr(4, 1);
CmA = coeff(5, 3);CmQ = coeff(5, 5);CmDe = coeff_ctr(5, 2);
Cn0 = yaw_moment_coeff;CnB = coeff(6, 6);CnR = coeff(6, 9);CnDr = coeff_ctr(6, 3);


%Note:leaving out moment arms becuase assuming CP and CG locations are same
%forces
D = (-CD0-abs(CDA*alpha))*q_bar*S;
Y = (CYB*beta)*q_bar*S;
L = (-CL0-CLA*alpha)*q_bar*S;

% stab2body = [cos(alpha), 0, -sin(alpha); 0, 1, 0; sin(alpha), 0, cos(alpha)];
% F_aero = stab2body*[D, Y, L]';
F_aero = [D, Y, L]';

%gravity
F_grav = dcm*gravity'*m;

%moments
ML = (ClP*p+ClDa*da)*q_bar*span*S;
MM = (CmA*alpha+CmQ*q+CmDe*de)*q_bar*chord*S;
MN = (Cn0+CnB*beta+CnR*r+CnDr*dr)*q_bar*span*S;
% M_aero = stab2body*[ML, MM, MN]';
M_aero = [ML, MM, MN]';

%moment mod
XYZ_body = dcm*[x, y, z]';
tmp = sqrt(XYZ_body(1)^2+XYZ_body(2)^2+XYZ_body(3)^2);
T_hat = -[XYZ_body(1)/tmp, XYZ_body(2)/tmp, XYZ_body(3)/tmp]';
%T_hat = -normalize(XYZ_body, 'norm');

tmp = sqrt(T_hat(2)^2+T_hat(3)^2);
T_inter_hat = [0, T_hat(2)/tmp, T_hat(3)/tmp]';
% T_inter_hat = normalize(T_hat.*[0, 1, 1]','norm');
y_hat = [0, 0, 1]';

phi_rel = acos(dot(y_hat, T_inter_hat)).*(T_inter_hat(2)/abs(T_inter_hat(2)));
%phi_rel = acos(dot(y_hat, T_inter_hat)).*normalize(T_inter_hat(2), 'norm');

theta_rel = acos(dot(T_inter_hat, T_hat)).*-(T_hat(1)/abs(T_hat(1)));
%theta_rel = acos(dot(T_inter_hat, T_hat)).*-normalize(T_hat(1), 'norm');
delta_roll = roll_c-phi_rel;
delta_pitch = pitch_c-theta_rel;
ML_C = (sin(delta_roll)*cos(delta_pitch))*roll_k;
MM_C = (sin(delta_pitch)*cos(delta_roll))*pitch_k;
MN_C = 0;
M_adcs = [ML_C, MM_C, MN_C]';

%sum forces and moments
F_out = F_aero+F_grav;
M_out = M_aero+M_adcs;


%solve for output accelerations for just aerodynamic model
% u_dot = F_out(1)/m;
% v_dot = F_out(2)/m;
% w_dot = F_out(3)/m;
% p_dot = M_out(1)/moi(1);
% q_dot = M_out(2)/moi(2);
% r_dot = M_out(3)/moi(3);
% 
% x_dot = [u, v, w, p, q, r, u_dot, v_dot, w_dot, p_dot, q_dot, r_dot];
end

