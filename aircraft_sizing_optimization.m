clc
close all
clear

%% Parameters
S = 1;
tether_angle = 30;
tether_cd = 1.05;
tether_yeild = 1300e6;
e = 0.05;
Cd0 = 0.05;

%density
rho_0 = 1.16;
h1 = 8330;

%wind
w_0ac = 5;
w_0 = 3/2*w_0ac;
h_0 = 10;
h_r = 0.1;

param = [rho_0; h1;w_0; h_0; h_r; tether_cd; tether_yeild; e; Cd0; S; tether_angle];

%Cl, h
ac_vals_guess = [1; 100];

%% Constraints
A = zeros(2);
A(1, 1) = 1;
b = [1; 0];

for i = 1.2:0.1:1.2
%     param(8) = i;
    b = [i; 0];

%% Optimize
[ac_vals, f0] = fmincon(@(ac_vals) objective(ac_vals, param), ac_vals_guess, A, b, [], [])

figure(1);
hold on;
scatter(i, ac_vals(1), 'blue');
xlabel("Maximum Cl")
ylabel("Optimial Cl")
title("Optimial Cl vs Maximum Cl")
legend("Optimal Cl")

figure(2);
hold on;
scatter(i, ac_vals(2), 'blue');
xlabel("Maximum Cl")
ylabel("Optimial Height [m]")
title("Height vs Maximum Cl")
legend("Optimal Height")

figure(3)
hold on;
scatter(i, -f0, 'blue');
xlabel("Maximum Cl")
ylabel("Power [W]")
title("Power [W] vs Maximum Cl")
legend("Optimal Power")
end



%% Cost Function

function J = objective(ac_vals, p)
%ac parameters
Cl = ac_vals(1);
h = ac_vals(2);

%constant parameters
rho_0 = p(1);
h1 = p(2);
w_0 = p(3);
h_0 = p(4);
h_r = p(5);
tether_cd = p(6);
tether_yeild = p(7);
e = p(8);
Cd0 = p(9);
S = p(10);
el = p(11);

%altitude fncs
%wind
V_w = ((log(h/h_r)/log(h_0/h_r)))*w_0;
%air density
rho = rho_0*exp(-h/h1);

%tether drag
Cd = Cd0+Cl^2*e;
for i = 1:7
V_a = 2*Cl*V_w/(3*Cd)
q = 0.5*rho*V_a^2;
L = q*S*Cl
D = q*S*Cd;
tether_r = sqrt(sqrt(L^2+D^2)/(pi*tether_yeild))
tether_l = h/sind(el)
C_dt_S = tether_cd*2*tether_r*tether_l;
C_dt = C_dt_S/(4*S)
Cl^2*e
Cd = Cd0+Cl^2*e+C_dt
end
tether_m = 1000*pi*tether_r^2*h/sind(el)
J = -((2/27)*rho*S*V_w^3*Cl*(Cl/Cd)^2)

end