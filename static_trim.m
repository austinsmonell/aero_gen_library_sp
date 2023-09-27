function [z_trim, f0] =  static_trim(x0, u0, p, targets)
    quat = eul2quat(x0(7:9)')';
    Z0 = [x0(1:6);quat;x0(10:12);u0];
    
    lb = -[Inf;Inf;Inf;Inf;Inf;Inf;Inf;Inf;Inf;Inf;Inf;Inf;Inf;0.8;0.8;0.8];
    ub = [Inf;Inf;Inf;Inf;Inf;Inf;Inf;Inf;Inf;Inf;Inf;Inf;Inf;0.8;0.8;0.8];
    z_trim = Z0;

    options = optimset('Display','iter');
    for i = 1:3
    [z_trim, f0] = fmincon(@(Z)trimObj(Z, p, targets),z_trim, [], [], [], [], lb, ub, [], []);
    clc;
    end
    z_trim = [z_trim(1:6); quat2eul(z_trim(7:10)')'; z_trim(11:16)];

end

%% Trim Objective Function
function cost = trimObj(z, p, targets)
    x = [z(1:6); quat2eul(z(7:10)')'; z(11:13)];
    u = z(14:16);
    dx = get_full_state(x, u, p);
    J = (x(3)-targets(1))^2 + (x(2)-targets(2))^2 + dx'*dx+x(7)^2;
    cost = J;
end

