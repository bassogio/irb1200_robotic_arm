function tau_static = staticTorquesForLoad(dh, jointAngles, M, g)
% staticTorquesForLoad
% Computes the static joint torques required to hold a load of weight M (kg)
% at the end-effector under gravity.
%
% Inputs:
%   dh          - struct of DH parameters (d1, a2, a3, d4, d6)
%   jointAngles - 6x1 vector of joint angles (radians)
%   M           - scalar, mass of the payload in kg
%   g           - scalar, gravitational acceleration (e.g., 9.81)
%
% Output:
%   tau_static  - 6x1 vector of required joint torques (Nm)

    % === Step 1: Compute the Jacobian at the given configuration ===
    [J, ~, ~] = Jacobian(dh, jointAngles);

    % === Step 2: Define the external wrench (force/torque at end-effector) ===
    F_ext = [0; 0; -M * g; 0; 0; 0];  % Only downward force in z

    % === Step 3: Compute joint torques for static equilibrium ===
    tau_static = J' * F_ext;

end
