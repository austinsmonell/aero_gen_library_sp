function flight_config = save_configuration(name,viz_on, ac1_forces, ac2_forces, ac1_moments, ac2_moments, WindVelocity, k_line, gravity, d, rho,...
                                                      pitch_c, k_pitch, pitch_moment_1, pitch_moment_2, roll_c_1, roll_c_2,k_roll, roll_moment_1, roll_moment_2, yaw_moment_1, yaw_moment_2, ...
                                                      InitEuler1, InitEuler2, InitPosition1, InitPosition2, InitPQR1, InitPQR2, InitVelocity1, InitVelocity2, InitAcc1, InitAcc2, InitTheta, InitThetaDot, ...
                                                      k_damp, c, moi, r, theta_0)

%% System Parameters
flight_config.name = name;
flight_config.sys_params.viz_on = viz_on;
flight_config.sys_params.ac1_forces = ac1_forces;
flight_config.sys_params.ac2_forces = ac2_forces;
flight_config.sys_params.ac1_moments = ac1_moments;
flight_config.sys_params.ac2_moments = ac2_moments;
flight_config.sys_params.wind = WindVelocity;
flight_config.sys_params.k_line = k_line;
flight_config.sys_params.gravity = gravity;
flight_config.sys_params.d = d;
flight_config.sys_params.rho = rho;

%% Aircraft Parameters
flight_config.aircraft_params.pitch_control_input = pitch_c;
flight_config.aircraft_params.pitch_k = k_pitch;
flight_config.aircraft_params.pitch_moment_coeff_1 = pitch_moment_1;
flight_config.aircraft_params.pitch_moment_coeff_2 = pitch_moment_2;
flight_config.aircraft_params.roll_control_input_1 = roll_c_1;
flight_config.aircraft_params.roll_control_input_2 = roll_c_2;
flight_config.aircraft_params.roll_k = k_roll;
flight_config.aircraft_params.roll_moment_coeff_1 = roll_moment_1;
flight_config.aircraft_params.roll_moment_coeff_2 = roll_moment_2;
flight_config.aircraft_params.yaw_moment_coeff_1 = yaw_moment_1;
flight_config.aircraft_params.yaw_moment_coeff_2 = yaw_moment_2;

%% Initial Conditions
flight_config.ic.euler_1 = InitEuler1;
flight_config.ic.euler_2 = InitEuler2;
flight_config.ic.position_1 = InitPosition1;
flight_config.ic.position_2 = InitPosition2;
flight_config.ic.pqr_1 = InitPQR1;
flight_config.ic.pqr_2 = InitPQR2;
flight_config.ic.velocity_1 = InitVelocity1;
flight_config.ic.velocity_2 = InitVelocity2;
flight_config.ic.acc_1 = InitAcc1;
flight_config.ic.acc_2 = InitAcc2;
flight_config.ic.theta_init = InitTheta;
flight_config.ic.theta_dot_init = InitThetaDot;

%% Generator Parameters
flight_config.gen_params.k_damp = k_damp;
flight_config.gen_params.c = c;
flight_config.gen_params.moi = moi;
flight_config.gen_params.r = r;
flight_config.gen_params.theta_0 = theta_0;
end