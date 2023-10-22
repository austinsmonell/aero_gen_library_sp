function [z_trim, f0] =  static_trim_full(x0, u0, p, targets)
    Z0 = [x0([1; 3; 8]);u0(2)];
    
    lb = -[Inf;Inf;Inf;0.8];
    ub = [Inf;Inf;Inf;0.8];
    z_trim = Z0;

%     options = optimoptions('fmincon', 'StepTolerance',1e-20, 'ConstraintTolerance', 1e-4, 'MaxIterations', 100000, 'OptimalityTolerance', 1e-4, 'MaxFunctionEvaluations', 100000, 'Display','iter');
    for i = 1:20
    [z_trim, f0] = fmincon(@(Z)trimObj(Z, x0, u0, p, targets),z_trim, [], [], [], [], lb, ub, [], []);
    end

end

%% Trim Objective Function
function cost = trimObj(z,x, u, p, targets)
    x = [z(1); x(2); z(2);x(4:7); z(3); x(9:12)];
    u = [u(1); z(4); u(3)];
    dx = get_full_state(x, u, p);
    J = dx'*dx;
    cost = J;
end

