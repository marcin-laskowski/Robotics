% ROBTOICS - tutorial 1

clc
clear all

% a = 20;
% b = 30;
% c = 5;
% d = 10;
% q1 = deg2rad(150);
% q2 = deg2rad(45);
% q3 = deg2rad(45);

syms q1 q2 q3 a b c d
%% Q1 // Forward Kinematics

SL = [0;0];	% point SL
SR = [d;0];
EL = [a*cos(q1); a*sin(q1)]; % point EL
ER = [d + a*cos(q2); a*sin(q2)]; % point ER

EL_ER = [(-1*EL(1)+ER(1)); (-1*EL(2)+ER(2))]; % vector EL_ER
mag_EL_ER = sqrt((EL_ER(1))^2+(EL_ER(2))^2); % magnitude of the vector EL_ER
unit_EL_ER = EL_ER / mag_EL_ER;	% unit vector EL_ER

EL_C = EL_ER / 2; % vector EL_C
mag_EL_C = sqrt((EL_C(1))^2+(EL_C(2))^2); % magnitude of the vector EL_C
C = SL + EL + EL_C;	% point C

mag_C_W = sqrt(b^2 - mag_EL_C^2); % magnitude of the vector C_w
unit_C_W = [-1 * unit_EL_ER(2); unit_EL_ER(1)];	% unit vector C_W
C_W = mag_C_W * unit_C_W; % vector C_W
W = C + C_W; % point W
    
W_H = [c*cos(q3); c*sin(q3)]; % vector W_H

H = W + W_H;
% simplify(H);
% pretty(H) % point H

%%%%% P L O T S %%%%%%
% figure(1)
% plot([SL(1),EL(1)],[SL(2),EL(2)]); hold on;
% plot([SL(1),SR(1)],[SL(2),SR(2)]); hold on;
% plot([SR(1),ER(1)],[SR(2),ER(2)]); hold on;
% plot([ER(1), W(1)], [ER(2), W(2)]); hold on;
% plot([EL(1), W(1)], [EL(2), W(2)]); hold on;
% plot([W(1), H(1)], [W(2), H(2)]); hold on;
% xlabel('x axis [m]')
% ylabel('y axis [m]')
% h = figure(1);
% saveas(h, 'plot1.png')
