function solutions = InverseKinematic(dh, T06)
% InverseKinematic  Compute all IK solutions for the ABB IRB1200.
%   Input:
%     dh  – struct with fields d1,a2,a3,d4,d6
%     T06 – 4×4 desired end-effector transform
%   Output:
%     solutions – array of structs with field .theta (6×1 vector)

  % Unpack DH constants
  d1 = dh.d1;  a2 = dh.a2;  a3 = dh.a3;
  d4 = dh.d4;  d6 = dh.d6;

  % Desired position & rotation
  p = T06(1:3,4);
  R = T06(1:3,1:3);

  % 1) Wrist center
  wc = p - d6 * R(:,3);
  Px = wc(1);  Py = wc(2);

  % 2) Two theta1 options
  th1_opts = [ atan2(Py,Px), atan2(-Py,-Px) ];

  solutions = [];
  for i1 = 1:2
    th1 = th1_opts(i1);

    % 3) theta2, theta3 via triangle law
    r = hypot(wc(1), wc(2));
    z = wc(3) - d1;
    C3 = (r^2 + z^2 - a2^2 - a3^2)/(2*a2*a3);
    if abs(C3) > 1
      continue;  % unreachable
    end
    s3_vals = [ sqrt(1-C3^2), -sqrt(1-C3^2) ];
    for k = 1:2
      th3 = atan2(s3_vals(k), C3);
      k1 = a2 + a3*C3;
      k2 = a3*s3_vals(k);
      th2 = atan2(z, r) - atan2(k2, k1);

      % 4) Compute R03
      [T03, ~, ~] = forwardKinematics(dh, [th1; th2; th3; 0; 0; 0]);
      R03 = T03(1:3,1:3);
      R36 = R03' * R;

      % 5) theta4, theta5, theta6 from R36
      r13 = R36(1,3);  r23 = R36(2,3);  r33 = R36(3,3);
      s5  = hypot(r13, r23);

      if s5 < 1e-6
        % Wrist singularity: choose th4 = 0, solve th6 from R36
        th5 = atan2(0, r33);  % = 0 or pi
        th4 = 0;
        th6 = atan2(-R36(1,2), R36(1,1));
        sol.theta = [th1; th2; th3; th4; th5; th6];
        solutions = [solutions; sol];
        continue;
      end

      th5_opts = [ atan2( s5, r33 ), atan2(-s5, r33) ];
      for j = 1:2
        th5 = th5_opts(j);
        denom = (j==1)*s5 + (j==2)*(-s5);
        th4 = atan2( R36(2,3)/denom, R36(1,3)/denom );
        th6 = atan2( R36(3,2)/denom, -R36(3,1)/denom );
        sol.theta = [th1; th2; th3; th4; th5; th6];
        solutions = [solutions; sol];
      end
    end
  end
end
