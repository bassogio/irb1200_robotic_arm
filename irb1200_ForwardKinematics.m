% clc;
% clear;
% close all;

% Declare symbolic variables
syms theta1 theta2 theta3 theta4 theta5 theta6
syms d1 d2 d3 d4 d6
assume([theta1 theta2 theta3 theta4 theta5 theta6],'real');
assume([d1 d2 d3 d4 d6],'real');

% Define the homogeneous transformation matrices  
T01 = [ cos(theta1)   0  -sin(theta1)   0;
        sin(theta1)   0   cos(theta1)   0;
             0       -1       0        d1;
             0        0       0         1 ];

T12 = [ sin(theta2)   cos(theta2)  0   d2*sin(theta2);
       -cos(theta2)   sin(theta2)  0  -d2*cos(theta2);
            0              0       1       0;
            0              0       0       1 ];

T23 = [ cos(theta3)   0  -sin(theta3)   d3*cos(theta3);
        sin(theta3)   0   cos(theta3)   d3*sin(theta3);
             0       -1       0         0;
             0        0       0         1 ];

T34 = [ cos(theta4)   0   sin(theta4)   0;
        sin(theta4)   0  -cos(theta4)   0;
            0         1       0        d4;
            0         0       0         1 ];

T45 = [ cos(theta5)   0  -sin(theta5)   0;
        sin(theta5)   0   cos(theta5)   0;
            0        -1       0         0;
            0         0       0         1 ];

T56 = [ -cos(theta6)   sin(theta6)   0    0;
        -sin(theta6)  -cos(theta6)   0    0;
             0             0         1   d6;
             0             0         0    1 ];

% Compute the full transformation matrix step-by-step  
T02 = simplify(T01 * T12);
T03 = simplify(T02 * T23);
T04 = simplify(T03 * T34);
T05 = simplify(T04 * T45);
T06 = simplify(T05 * T56);

% % Apply trigonometric identity simplification  
% T06_simplified = simplify(T06, 'Criterion', 'preferReal', 'Steps', 100);
% 
% % Display symbolic result  
% disp('The simplified symbolic transformation matrix T06 is:');
% disp(T06_simplified);
