clc;
clear;
close all;

%% Define DH parameters (units in mm and degrees)
d1 = 0.399; a2 = 0.350; a3 = 0.042; d4 = 0.351; d6 = 0.082;

%      [theta          ,    d ,    a ,    alpha ]
DH = [ theta1          ,   d1 ,    0 ,   -pi/2  ;
       theta2 - pi/2   ,    0 ,   a2 ,    0     ;
       theta3          ,    0 ,   a3 ,   -pi/2  ;
       theta4          ,   d4 ,    0 ,    pi/2  ;
       theta5          ,    0 ,    0 ,   -pi/2  ;
       theta6 - pi     ,   d6 ,    0 ,    0     ];
