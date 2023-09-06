function [x_dot] = get_simple_state(x_in, const, t, T, alpha_max, alpha_min, wind)
%% Assumtions:
%1:Alpha can be obtained no matter the condition
%2:Lift is always alligned with tether
%3:Kite aero forces directly applied to generator (no elastic forces)
%4:wind is applied by aero force opposing drag force


%% Alpha Law
alpha1 = (alpha_max-alpha_min)/2*sin(2*pi/T*t+pi/2)+(alpha_max-alpha_min)/2+alpha_min;
alpha2 = (alpha_max-alpha_min)/2*sin(2*pi/T*t-pi/2)+(alpha_max-alpha_min)/2+alpha_min;
if(sin(2*pi/T*t+pi/2)>0)
    mode = 1;
else
    mode = 2;
end
%% Extract States
ac1_state = x_in(1);
ac2_state = x_in(2);
gen_state = x_in(3:4);

va_1 = ac1_state(1);
va_2 = ac2_state(1);
theta = gen_state(1); theta_dot = gen_state(2);


%% Extract Constants
m = const(1); gen_moi = const(2); gen_c = const(3);
l_slope = const(4); d_slope = const(5); d_0 = const(6);
rho = const(7); s = const(8); gen_r = const(9);

%% Lift and Drag AC1
q_1 = 0.5*rho*(va_1^2);
cl_1 = l_slope*alpha1;
if(theta_dot > 0)
L1 = q_1*cl_1*s-theta_dot*gen_c;
else
L1 = q_1*cl_1*s;
end

cd_1 = d_slope*alpha1+d_0;
D1 = q_1*cd_1*s;

%% Lift and Drag AC2
q_2 = 0.5*rho*(va_2^2);
cl_2 = l_slope*alpha2;
if(theta_dot < 0)
L2 = q_2*cl_2*s+theta_dot*gen_c;
else
L2 = q_2*cl_2*s;
end

cd_2= d_slope*alpha2+d_0;
D2 = q_2*cd_2*s;

%% Gen Moment
M = (L1-L2)*gen_r;


%% Get Accelerations
W1 = (0.5*rho*wind^2)*cl_1*s*sin(alpha1);
W2 = (0.5*rho*wind^2)*cl_2*s*sin(alpha2);

va_1_dot = (-D1+W1)/m;
va_2_dot = (-D2+W2)/m;
theta_dot_dot = M/gen_moi;

%% Output Derivtives
x_dot = [va_1_dot,...
         va_2_dot, ...
         theta_dot, theta_dot_dot]';
end