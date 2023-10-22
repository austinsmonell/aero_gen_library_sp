function [t, x_out] = run_sim_states(Tstop, x0,u0, param)
addpath('../aero_gen_library_sp/');

%Continous
tspan = [0, Tstop];
[t, x_out] = ode45(@(t, x) get_full_state(x, u0, param), tspan, x0);
end