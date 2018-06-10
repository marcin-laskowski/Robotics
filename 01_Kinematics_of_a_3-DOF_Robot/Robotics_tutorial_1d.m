%% Movement integration
clc
clear all

a = 0.3;
c = 0.2;
T = 2;
t = linspace(0,T,100);
tau = t/T;

H1 = [0.141, 0.441];
H2 = [0.241, 0.641];
X = H2(1) - H1(1);
Y = H2(2) - H1(2);
trajectory = sqrt((X)^2+(Y)^2);
sigma = (30.*tau.^2).*(tau.^2-2*tau+1);

velocity_x = (sigma*trajectory) * (X/trajectory);
velocity_y = (sigma*trajectory) * (Y/trajectory);
velocity = sigma * trajectory;

dX = [velocity_x; velocity_y];
B = dX';

q1 = deg2rad(150);
q2 = deg2rad(30);
q3 = deg2rad(45);
q1_0 = deg2rad(150);
q2_0 = deg2rad(30);
q3_0 = deg2rad(45);

syms dq1 dq2 dq3 
dQ = [];

%%

for i = 2:100
    J = [-a*sin(q1), -a*sin(q2), -c*sin(q3); a*cos(q1), a*cos(q2), c*cos(q3)];
    dQ(:,i) = pinv(J) * B(i,:)';

    time = linspace(0,1,100);
    Q1(i) = trapz(time(1:i), dQ(1, 1:i)) + q1_0;
    Q2(i) = trapz(time(1:i), dQ(2, 1:i)) + q2_0;
    Q3(i) = trapz(time(1:i), dQ(3, 1:i)) + q3_0;

    q1 = Q1(i);
    q2 = Q2(i);
    q3 = Q3(i);
   
end


figure(1)
plot(t, rad2deg(Q1))
% title('qL profile')
xlabel('time [s]')
ylabel('position [\circ]')

figure(2)
plot(t, rad2deg(Q2))
% title('qR profile')
xlabel('time [s]')
ylabel('position [\circ]')

figure(3)
plot(t, rad2deg(Q3))
% title('theta profile')
xlabel('time [s]')
ylabel('position [\circ]')
