
function [position, T06_eval] = forward_kinematics_Guy(D, A, Theta, Alpha)
    % joint_angles: 1x6 vector [theta1, ..., theta6] in radians

    len = length(D);
    T = {};

    % Declare symbolic variables
    syms ('theta', 'd', 'a', 'alph',[1 len],'real');

    % Define transformation matrices (symbolic)
    for i = 1:1:len
        T.(['T' num2str(i-1) num2str(i)]) = DH_Transform(d(i),a(i),theta(i),alph(i));
    end

    % Define transformation matrices (symbolic)
    
    T01 = DH_Transform(d(1),a(1),theta(1),alph(1));
    T12 = DH_Transform(d(2),a(2),theta(2),alph(2));
    T23 = DH_Transform(d(3),a(3),theta(3),alph(3));
    T34 = DH_Transform(d(4),a(4),theta(4),alph(4));
    T45 = DH_Transform(d(5),a(5),theta(5),alph(5));
    T56 = DH_Transform(d(6),a(6),theta(6),alph(6));
    T67 = DH_Transform(d(7),a(7),theta(7),alph(7));

    % Full symbolic transformation
    for i = 2:1:len
        T.(['T0' num2str(i)]) = T.(['T0' num2str(i-1)]) * T.(['T' num2str(i-1) num2str(i)]);
    end

    T02 = T01 * T12;
    T03 = T01 * T12 * T23;
    T04 = T01 * T12 * T23 * T34;
    T05 = T01 * T12 * T23 * T34 * T45;
    T06 = T01 * T12 * T23 * T34 * T45 * T56;
    T07 = T01 * T12 * T23 * T34 * T45 * T56 * T67;

    % Apply numeric joint values and constants
    D_vals = num2cell(D);
    A_vals = num2cell(A);
    Theta_vals = num2cell(Theta);
    Alpha_vals = num2cell(Alpha);
    
    symbols =  transpose([d(:); a(:); theta(:); alph(:)]);
    values =  [D_vals, A_vals, Theta_vals, Alpha_vals];

    for i = 1:1:len
        T_eval.(['T0' num2str(i)]) = double(subs(T.(['T0' num2str(i)]), symbols, values));
        % display(T_eval.(['T0' num2str(i)]));
    end

    T01_eval = double(subs(T01, symbols, values))
    T02_eval = double(subs(T02, symbols, values))
    T03_eval = double(subs(T03, symbols, values))
    T04_eval = double(subs(T04, symbols, values))
    T05_eval = double(subs(T05, symbols, values))
    T06_eval = double(subs(T06, symbols, values))
    T07_eval = double(subs(T07, symbols, values))

    % Extract position
    position = T07_eval(1:3, 4);
end

function T=DH_Transform(d, a, theta, alph)
T = [cos(theta), -sin(theta)*cos(alph), sin(theta)*sin(alph), a*cos(theta);
    sin(theta), cos(theta)*cos(alph), -cos(theta)*sin(alph), a*sin(theta);
    0, sin(alph), cos(alph), d;
    0, 0, 0, 1];
end