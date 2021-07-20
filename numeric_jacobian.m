function jac = numeric_jacobian(forward_kinematics,puma, x)
% Calculate Jacobian of function f at given x

epsilon = 1e-6; 
epsilon_inv = 1/epsilon;
nx = length(x); % Dimension of the input x;
f0 = feval(forward_kinematics,puma, x); % caclulate f0, when no perturbation happens

% Do perturbation
for i = 1 : nx
    x_ = x;
    x_(i) =  x(i) + epsilon;
    jac(:, i) = (feval(forward_kinematics,puma, x) - f0) .* epsilon_inv;
end

    


