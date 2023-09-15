function [state1] = set_parameters(WindVelocity,InitPosition1, InitVelocity1,InitEuler1, InitPQR1, state1)
state1.Environment.WindVelocity = WindVelocity;
state1.XN = InitPosition1(1);
state1.XE = InitPosition1(2);
state1.XD = InitPosition1(3);
state1.U = InitVelocity1(1);
state1.V = InitVelocity1(2);
state1.W = InitVelocity1(3);
state1.Phi = InitEuler1(1);
state1.Theta = InitEuler1(2);
state1.Psi = InitEuler1(3);
state1.P = InitPQR1(1);
state1.Q = InitPQR1(2);
state1.R = InitPQR1(3);

end