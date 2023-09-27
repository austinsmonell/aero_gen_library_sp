clc
close all
clear

%% Parameters
S_max = 1;
tether_angle = deg2rad(45);
tether_cd = 1;
tether_yeild = 850e6;
e = 0.7;
ar = 3;
Cd0 = 0.02;

%density
rho_0 = 1.14;
h1 = 8330;

%wind
w_0 = 6;
h_0 = 2;
h_r = 0.1;

param = [rho_0; h1;w_0; h_0; h_r; tether_angle; tether_cd; tether_yeild; e; ar; Cd0];

%S, Cl, h
ac_vals_guess = [0.5; 5; 100];

%% Constraints
A = zeros(3);
A(1, 1) = 1;A(2, 2) = 1;
b = [S_max; 2; 0];



%% Optimize
[ac_vals, f0] = fmincon(@(ac_vals) objective(ac_vals, param), ac_vals_guess, A, b, [], [])

%% Cost Function

function J = objective(ac_vals, p)
%ac parameters
S = ac_vals(1);
Cl = ac_vals(2);
h = ac_vals(3);

%constant parameters
rho_0 = p(1);
h1 = p(2);
w_0 = p(3);
h_0 = p(4);
h_r = p(5);
tether_angle = p(6);
tether_cd = p(7);
tether_yeild = p(8);
e = p(9);
ar = p(10);
Cd0 = p(11);

%altitude fncs
%wind
V_w = ((log(h/h_r)/log(h_0/h_r)))*w_0;
%air density
rho = rho_0*exp(-h/h1);

%tether drag
Cd = Cd0+Cl^2/(pi*e*ar);
for i = 1:7
V_a = 2*Cl*V_w/(3*Cd);
q = 0.5*rho*V_a^2;
L = q*S*Cl;
tether_r = sqrt(L/(pi*tether_yeild));
C_dt_S = tether_cd*2*tether_r*h/sin(tether_angle);
C_dt = C_dt_S/S;
Cd = Cd0+Cl^2/(pi*e*ar)+C_dt/8
end

J = -((2/27)*rho*S*V_w^3*Cl*(Cl/Cd)^2);

end