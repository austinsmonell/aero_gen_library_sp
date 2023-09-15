function [t, y_out] = run_sim_states(Tstop, ac1_state,ctr1, param)
addpath('../aero_gen_library_sp/');

% Discrete
% time = 0:Ts:Tstop;
% for idx = 1:(Tstop/Ts)+1
% t = (idx-1)*Ts;
% out_states(idx, :) = [ac1_state, ac2_state, gen_state];
% [x_dot_ac1, x_dot_ac2, x_dot_gen] = gen_full_state(ac1_state, ac2_state, gen_state, ctr1, ctr2, const, coeff,coeff_ctr, yaw_moment_1, yaw_moment_2, pitch_c, roll_c_1, roll_c_2);
% ac1_state = ac1_state+x_dot_ac1*Ts;
% ac2_state = ac2_state+x_dot_ac2*Ts;
% gen_state = gen_state+x_dot_gen*Ts;
% end

%Continous
tspan = [0, Tstop];
y0 = [ac1_state];
u0 = [ctr1];
[t, y_out] = ode45(@(t, y_in) get_full_state(y_in, u0, param), tspan, y0);


end