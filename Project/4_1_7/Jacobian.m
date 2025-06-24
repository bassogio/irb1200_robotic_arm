function J = Jacobian(dh, thetas)
    % Get origins p_i and rotation matrices R_i from ForwardKinematics
    [~, origins, rots] = forwardKinematics(dh, thetas);

    pe = origins(:,end);          % end-effector origin
    Jv = zeros(3,6);
    Jw = zeros(3,6);

    for i = 1:6
        zi = rots(:,3,i);         % z-axis of joint i (3rd column)
        pi = origins(:,i);        % origin of joint i
        Jv(:,i) = cross(zi, pe - pi);
        Jw(:,i) = zi;
    end

    J = [Jv; Jw];                 % 6×6 Jacobian
end
