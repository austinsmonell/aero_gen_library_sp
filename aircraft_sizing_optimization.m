clc
close all
clear

%% Parameters
rho = 1.225;
A_max = 1;
V_w_ground_min = 10;

param = [rho;V_w_ground_min];

ac_vals_guess = [0.5; 1.2; 0.05];

%% Constraints
A = eye(3);
A(3, 3) = -1;
b = [A_max; 1; -0.1];



%% Optimize
[ac_vals, f0] = fmincon(@(ac_vals) objective(ac_vals, param), ac_vals_guess, A, b, [], [])

%% Cost Function

function J = objective(ac_vals, p)
%constant parameters
rho = p(1);
V_w_ground = p(2);

%ac parameters
A = ac_vals(1);
Cl = ac_vals(2);
Cd = ac_vals(3);
V_w = V_w_ground;

J = -((2/27)*rho*A*V_w^3*Cl*(Cl/Cd)^2);

end