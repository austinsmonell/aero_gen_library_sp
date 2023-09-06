clc
clear
close all
%       ac_m, moi, c,  a_l,a_d,  d_0,   rho,  S, gen_r 
const = [2, 0.01, .1, 6.2, 0.1, 0.021, 1.225, .6, 0.1];
T = 10;
alpha_max = deg2rad(10);
alpha_min = deg2rad(0);
wind = 10;
y0 = [10, 10, 0, 0];
Tstop = 500;
tspan = [0, Tstop];
[t, y_out] = ode45(@(t, y_in) get_simple_state(y_in, const, t, T, alpha_max, alpha_min, wind), tspan, y0);

ac1_state = y_out(:, 1);
ac2_state = y_out(:, 2);
gen_state = y_out(:, 3:4);

va_1 = ac1_state(:, 1);
va_2 = ac2_state(:, 1);
theta = gen_state(:, 1); theta_dot = gen_state(:, 2);
power = abs((theta(2:end)-theta(1:end-1)).*theta_dot(2:end)*const(3))./(t(2:end)-t(1:end-1));

%% Plot
subplot(2, 3, 1);
plot(t, va_1);
ylabel('Airspeed 1 [mps]');
subplot(2, 3, 2);
plot(t, theta);
ylabel('Theta [rad]');
subplot(2, 3, 3);
plot(t, theta_dot);
ylabel('Theta dot [rad/s]');
subplot(2, 3, 4);
plot(t, va_2);
ylabel('Airspeed 2 [mps]');
subplot(2, 3, 5);
plot(t, theta*const(5));
ylabel('Distance [m]');
subplot(2, 3, 6);
plot(t(2:end), power/1000);
ylabel('Power [kW]');

%% Analysis

for i = 0:(Tstop-T)
t_init = i;
time_window = find(t>t_init & t<(t_init+T));
energy(i+1) = trapz(t(time_window), power(time_window));
end

figure(2);
plot(0:(Tstop-T), energy);