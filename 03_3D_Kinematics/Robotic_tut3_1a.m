%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Robotic_tut3_1a
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Denavit-Hartenberg Notation
%RotZ = [cos(theta) sin(theta) 0 0; sin(theta) cos(theta) 0 0; 0 0 1 0; 0 0 0 1]
%TransZ = [1 0 0 0; 0 1 0 0; 0 0 1 d; 0 0 0 1]
%TransX = [1 0 0 a; 0 1 0 0; 0 0 1 0; 0 0 0 1]
%RotX = [1 0 0 0; 0 cos(alpha) -sin(alpha) 0; 0 sin(alpha) cos(alpha) 0; 0 0 0 1];
% A = RotZ * TransZ * TransX * RotX;

syms theta1 theta2 theta3 theta4 theta5 theta6 theta7
syms alpha1 alpha2 alpha3 alpha4 alpha5 alpha6 alpha7
syms a1 a2 a3 a4 a5 a6 a7
syms d1 d2 d3 d4 d5 d6 d7

% a1 = 0;         alpha1 = 0;         d1 = 0.2703;
% a2 = 0.069;     alpha2 = -1.571;    d2 = 0;
% a3 = 0;         alpha3 = 1.571;     d3 = 0.3644;
% a4 = 0.069;     alpha4 = -1.571;    d4 = 0;
% a5 = 0;         alpha5 = 1.571;     d5 = 0.3743;
% a6 = 0.01;      alpha6 = -1.571;    d6 = 0;
% a7 = 0;         alpha7 = 1.571;     d7 = 0.2295;

A1 = [1 0 0 0; 0 1 0 0; 0 0 1 d1; 0 0 0 1]*[cos(theta1) -sin(theta1) 0 0; sin(theta1) cos(theta1) 0 0; 0 0 1 0; 0 0 0 1];
A2 = [1 0 0 a2; 0 1 0 0; 0 0 1 0; 0 0 0 1]*[1 0 0 0; 0 cos(alpha2) -sin(alpha2) 0; 0 sin(alpha2) cos(alpha2) 0; 0 0 0 1]*[cos(theta2) -sin(theta2) 0 0; sin(theta2) cos(theta2) 0 0; 0 0 1 0; 0 0 0 1];
A3 = [1 0 0 0; 0 cos(alpha3) -sin(alpha3) 0; 0 sin(alpha3) cos(alpha3) 0; 0 0 0 1]*[1 0 0 0; 0 1 0 0; 0 0 1 d3; 0 0 0 1]*[cos(theta3) -sin(theta3) 0 0; sin(theta3) cos(theta3) 0 0; 0 0 1 0; 0 0 0 1];
A4 = [1 0 0 a4; 0 1 0 0; 0 0 1 0; 0 0 0 1]*[1 0 0 0; 0 cos(alpha4) -sin(alpha4) 0; 0 sin(alpha4) cos(alpha4) 0; 0 0 0 1]*[cos(theta4) -sin(theta4) 0 0; sin(theta4) cos(theta4) 0 0; 0 0 1 0; 0 0 0 1];
A5 = [1 0 0 0; 0 cos(alpha5) -sin(alpha5) 0; 0 sin(alpha5) cos(alpha5) 0; 0 0 0 1]*[1 0 0 0; 0 1 0 0; 0 0 1 d5; 0 0 0 1]*[cos(theta5) -sin(theta5) 0 0; sin(theta5) cos(theta5) 0 0; 0 0 1 0; 0 0 0 1];
A6 = [1 0 0 a6; 0 1 0 0; 0 0 1 0; 0 0 0 1]*[1 0 0 0; 0 cos(alpha6) -sin(alpha6) 0; 0 sin(alpha6) cos(alpha6) 0; 0 0 0 1]*[cos(theta6) -sin(theta6) 0 0; sin(theta6) cos(theta6) 0 0; 0 0 1 0; 0 0 0 1];
A7 = [1 0 0 0; 0 cos(alpha7) -sin(alpha7) 0; 0 sin(alpha7) cos(alpha7) 0; 0 0 0 1]*[1 0 0 0; 0 1 0 0; 0 0 1 d7; 0 0 0 1]*[cos(theta7) -sin(theta7) 0 0; sin(theta7) cos(theta7) 0 0; 0 0 1 0; 0 0 0 1];

% homogeneous transformation matrix
T = A1 * A2 * A3 * A4 * A5 * A6 * A7;
 
% subs(T,{a1,a2,a3,a4,a5,a6,a7},{0,0.069,0,0.069,0,0.01,0})
% T = ans;
% subs(T,{d1,d2,d3,d4,d5,d6,d7},{0.2703,0,0.3644,0,0.3743,0,0.2295})
% T = ans;
% subs(T,{alpha1,alpha2,alpha3,alpha4,alpha5,alpha6,alpha7},{0,-1.571,1.571,-1.571,1.571,-1.571,1.571})
% T = ans;
% subs(T,{theta1,theta2,theta3,theta4,theta5,theta6,theta7},{0,-0.55,0,1.2840,0,0.2616,0})
% T = ans;
