%% Q2_B Movement integration

% PLOT OF THE HAND VELOCITY
T = 2;
H1 = [0.141, 0.441]';
H2 = [0.241, 0.641]';
X = H2(1) - H1(1);
Y = H2(2) - H1(2);
trajectory = sqrt((X)^2 + (Y)^2);

t = linspace(0,T,100);
tau = t/T;
vel = (30.*tau.^2).*(tau.^2-2*tau+1);
vel = vel'*trajectory;

figure(1);
plot(t, vel);
xlabel('time [s]');
ylabel('velocity [m/s]');
fig1 = figure(1);
% saveas(fig1, 'hand_velocity.png')


% PLOT OF THE HAND POSITION PROFILE
hand = 6.*tau.^5-15.*tau.^4+10.*tau.^3;
hand_pos = hand * trajectory;
figure(2);
plot(t, hand_pos);
xlabel('time [s]');
ylabel('velocity [m/s^2]');
fig2 = figure(2);
% saveas(fig2, 'hand_position.png')


% PLOT OF THE HAND TRAJCETORY

hand_pos_X = H1(1) + (hand_pos) * X/trajectory
hand_pos_Y = H1(2) + (hand_pos) * Y/trajectory

figure(3)
plot(hand_pos_X, hand_pos_Y);
xlabel('x axis [m]');
ylabel('y axis [m]');
hand_trajectory = figure(3);
% saveas(hand_trajectory, 'hand_trajectory.png');
