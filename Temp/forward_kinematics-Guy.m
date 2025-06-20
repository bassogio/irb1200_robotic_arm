
function [position, T06_eval] = forward_kinematics_Guy(D, A, Theta, Alpha)
    % joint_angles: 1x6 vector [theta1, ..., theta6] in radians

    % Declare symbolic variables
    syms ('theta', 'd', 'a', 'alph',[1 6],'real');
    
    % Define transformation matrices (symbolic)
    
    T01 = DH_Transform(d(1),a(1),theta(1),alph(1));
    T12 = DH_Transform(d(2),a(2),theta(2),alph(2));
    T23 = DH_Transform(d(3),a(3),theta(3),alph(3));
    T34 = DH_Transform(d(4),a(4),theta(4),alph(4));
    T45 = DH_Transform(d(5),a(5),theta(5),alph(5));
    T56 = DH_Transform(d(6),a(6),theta(6),alph(6));

    % Full symbolic transformation
    T06 = T01 * T12 * T23 * T34 * T45 * T56;

    % Apply numeric joint values and constants
    D_vals = num2cell(D);
    A_vals = num2cell(A);
    Theta_vals = num2cell(Theta);
    Alpha_vals = num2cell(Alpha);
    
    symbols =  transpose([d(:); a(:); theta(:); alph(:)]);
    values =  [D_vals, A_vals, Theta_vals, Alpha_vals];
    
    T01_eval = double(subs(T01, symbols, values))
    T12_eval = double(subs(T12, symbols, values))
    T23_eval = double(subs(T23, symbols, values))
    T34_eval = double(subs(T34, symbols, values))
    T45_eval = double(subs(T45, symbols, values))
    T56_eval = double(subs(T56, symbols, values))
    T06_eval = double(subs(T06, symbols, values))

    % Extract position
    position = T06_eval(1:3, 4);
end

function T=DH_Transform(d, a, theta, alph)
T = [cos(theta), -sin(theta)*cos(alph), sin(theta)*sin(alph), a*cos(theta);
    sin(theta), cos(theta)*cos(alph), -cos(theta)*sin(alph), a*sin(theta);
    0, sin(alph), cos(alph), d;
    0, 0, 0, 1];
end