%% Q2 - CONTROL

clear all
clc

syms q1 q2 
m = 1; %[kg]  mass
l = 0.2; %[m]  length of the link
l_m = 0.1; %[m] distance from joint to center of mass
I = 0.01; %[kg/m^2] 

alpha = 2*m*l_m^2 + m*l^2 + 2*I;
beta = 2*m*l*l_m*cos(q2-q1);
H = [alpha, beta; beta alpha]; % mass distribution matrix

dt = 1000;
T = 2;
t = 0:T/dt:T;
w = t/T;
% trajectory vector
xd = [(0.273 - 0.2*(6*w.^5 - 15*w.^4 + 10*w.^3)); (0.273 - 0.1*(6*w.^5 - 15*w.^4 + 10*w.^3))];
% velocity vector
dxd = [(-3*(w.^4-2*w.^3+w.^2)); ((-1.5*(w.^4-2*w.^3+w.^2)))];
B = dxd';
% acceleration vector
ddxd = [(-1.5*(4*w.^3 - 6*w.^2 + 2*w)); ((-0.75*(4*w.^3 - 6*w.^2 + 2*w)))];

q1 = deg2rad(60);
q2 = deg2rad(30);
q1_0 = deg2rad(60);
q2_0 = deg2rad(30);

dQ = [];
Q1 = [];
Q2 = [];
ddQ1 = [];
ddQ1 = [];


for i = 1:dt
    
    % VELOCITY - DESIRED dQ
    J = l*[-sin(q1), -sin(q2); cos(q1), cos(q2)];
    dQ(:,i) = pinv(J) * B(i,:)';
    dQ1(i) = dQ(1,i);
    dQ2(i) = dQ(2,i);
    
    % POSITION - DESIRED Q
    Q1(i) = trapz(dQ1(1:i))*T/dt + q1_0;
    Q2(i) = trapz(dQ2(1:i))*T/dt + q2_0;

    q1 = Q1(i);
    q2 = Q2(i);
       
end

for i = 2:dt
   
    % ACCELERATION - DESIRED Q
    ddQ1(i) = (dQ1(i) - dQ1(i-1))/ T/dt;
    ddQ2(i) = (dQ2(i) - dQ2(i-1))/ T/dt;
    
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% FEEDBACK

new_Q1 = zeros(1,dt);
new_Q2 = zeros(1,dt);
new_dQ1 = zeros(1,dt);
new_dQ2 = zeros(1,dt);
new_ddQ1 = zeros(1,dt);
new_ddQ2 = zeros(1,dt);

new_Q1(1) = Q1(1);
new_Q2(1) = Q2(1);
new_dQ1(1) = dQ1(1);
new_dQ2(1) = dQ2(1);
new_ddQ1(1) = ddQ1(1);
new_ddQ2(1) = ddQ2(1);
new_Q1(2) = Q1(2);
new_Q2(2) = Q2(2);
new_dQ1(2) = dQ1(2);
new_dQ2(2) = dQ2(2);
new_ddQ1(2) = ddQ1(2);
new_ddQ2(2) = ddQ2(2);


tau1 = [];
tau2 = [];
new_ddQ = [];

K = 0.01; %[Nm]
k = 100; %[s]

Xfb(1,1) = xd(1,1);
Yfb(1,1) = xd(2,1);
Xff(1,1) = xd(1,1);
Yff(1,1) = xd(2,1);

for i = 2:dt
     
    % TAU
    e1 = Q1(i) - new_Q1(i);
    e2 = Q2(i) - new_Q2(i);
    de1 = dQ1(i) - new_dQ1(i);
    de2 = dQ2(i) - new_dQ2(i);

    tau1 = K*(e1 + k*de1);
    tau2 = K*(e2 + k*de2);
    Tau(i,:) = [tau1, tau2];
    
    
    % NEW ACCELERATION - ddQ
    alpha = 2*m*l_m^2 + m*l^2 + 2*I;
    beta = 2*m*l*l_m*cos(new_Q2(i)-new_Q1(i));
    H = [alpha, beta; beta alpha];
    new_H = pinv(H);
    C = 2*m*l*l_m*sin(new_Q2(i)-new_Q1(i));
    
    part_of_eq(i,:) = Tau(i,:) - C*[-(new_dQ2(i))^2; (new_dQ1(i))^2]';
    new_ddQ(i,:) = new_H * part_of_eq(i,:)';
    new_ddQ1(i) = new_ddQ(i,1);
    new_ddQ2(i) = new_ddQ(i,2);
    new_ddQ1(i+1) = new_ddQ(i,1);
    new_ddQ2(i+1) = new_ddQ(i,2);
    
    
    % REAL VELOCTY
    new_dQ1(i+1) = new_dQ1(i) + new_ddQ1(i)*(T/dt);
    new_dQ2(i+1) = new_dQ2(i) + new_ddQ2(i)*(T/dt);

   
    % REAL POSITION
    new_Q1(i+1) = new_Q1(i) + new_dQ1(i)*(T/dt);
    new_Q2(i+1) = new_Q2(i) + new_dQ2(i)*(T/dt);
    
    % ACTUAL X AND Y 
    Xfb(1,i) = l*cos(new_Q2(1,i))+l*cos(new_Q1(1,i));
    Yfb(1,i) = l*sin(new_Q2(1,i))+l*sin(new_Q1(1,i));
    
    % ROBOT POSITION
    RIGHTarm_fb(:,i)=l*[cos(new_Q2(1,i));sin(new_Q2(1,i))];
    LEFTarm_fb(:,i)=l*[cos(new_Q1(1,i));sin(new_Q1(1,i))];
        
    
    
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% FEEDBACK + FEEDFORWARD

TauFB = Tau';

new2_Q1 = zeros(1,dt);
new2_Q2 = zeros(1,dt);
new2_dQ1 = zeros(1,dt);
new2_dQ2 = zeros(1,dt);
new2_ddQ1 = zeros(1,dt);
new2_ddQ2 = zeros(1,dt);

new2_Q1(1) = Q1(1);
new2_Q2(1) = Q2(1);
new2_dQ1(1) = dQ1(1);
new2_dQ2(1) = dQ2(1);
new2_ddQ1(1) = ddQ1(1);
new2_ddQ2(1) = ddQ2(1);
new2_Q1(2) = Q1(2);
new2_Q2(2) = Q2(2);
new2_dQ1(2) = dQ1(2);
new2_dQ2(2) = dQ2(2);
new2_ddQ1(2) = ddQ1(2);
new2_ddQ2(2) = ddQ2(2);

new2_tau1 = [];
new2_tau2 = [];
new2_TauFB = [];


for i = 2:dt
     
    % TAU
    J_dot = l*[-cos(Q1(i))*dQ1(i), - cos(Q2(i))*dQ2(i); -sin(Q1(i))*dQ1(i), -sin(Q2(i))*dQ2(i)];
    J = l*[-sin(Q1(i)), -sin(Q2(i)); cos(Q1(i)), cos(Q2(i))];
    J = pinv(J);
   
    feedback(:,i) = J * ([ddxd(1,i);ddxd(2,i)] - J_dot*[dQ1(1,i); dQ2(1,i)]);
       
    alpha_2 = 2*m*l_m^2 + m*l^2 + 2*I;
    beta_2 = 2*m*l*l_m*cos(Q2(i)-Q1(i));
    H_des = [alpha_2, beta_2; beta_2 alpha_2];
    tauFF1 = H_des(1,1)*feedback(1,i)+H_des(1,2)*feedback(2,i) + 2*m*l*l_m*sin(Q2(i)-Q1(i))*(-(dQ2(i))^2);
    tauFF2 = H_des(2,1)*feedback(1,i)+H_des(2,2)*feedback(2,i) + 2*m*l*l_m*sin(Q2(i)-Q1(i))*((dQ1(i))^2);
    TauFF(:,i) = [tauFF1; tauFF2];
     

    % ERROR
    new2_e1 = Q1(i) - new2_Q1(i);
    new2_e2 = Q2(i) - new2_Q2(i);
    new2_de1 = dQ1(i) - new2_dQ1(i);
    new2_de2 = dQ2(i) - new2_dQ2(i);
    
    new2_tau1 = K*(new2_e1 + k*new2_de1);
    new2_tau2 = K*(new2_e2 + k*new2_de2);
    new2_TauFB(:,i) = [new2_tau1, new2_tau2];
    
    
    TauFinal(:,i) = TauFF(:,i) + new2_TauFB(:,i);
    TauFinalT = TauFinal';
    
    % NEW ACCELERATION - ddQ
    alpha_3 = 2*m*l_m^2 + m*l^2 + 2*I;
    beta_3 = 2*m*l*l_m*cos(new2_Q2(i)-new2_Q1(i));
    H = [alpha_3, beta_3; beta_3 alpha_3];
    new_H = pinv(H);
    delta = new_H(1,1);
    gamma = new_H(1,2);
    C = 2*m*l*l_m*sin(new2_Q2(i)-new2_Q1(i));
    
    new2_Tau(i,:) = TauFinalT(i,:) - C*[-(new2_dQ2(i))^2; (new2_dQ1(i))^2]';
    new2_ddQ(i,:) = new_H * new2_Tau(i,:)';
    new2_ddQ1(i) = new2_ddQ(i,1);
    new2_ddQ2(i) = new2_ddQ(i,2);
    
    
    % REAL VELOCTY
    new2_dQ1(i+1) = new2_dQ1(i) + new2_ddQ1(i)*(T/dt);
    new2_dQ2(i+1) = new2_dQ2(i) + new2_ddQ2(i)*(T/dt);

   
    % REAL POSITION
    new2_Q1(i+1) = new2_Q1(i) + new2_dQ1(i)*(T/dt);
    new2_Q2(i+1) = new2_Q2(i) + new2_dQ2(i)*(T/dt);
      
    
    % ACTUAL X AND Y 
    Xff(1,i) = l*cos(new2_Q2(1,i))+l*cos(new2_Q1(1,i));
    Yff(1,i) = l*sin(new2_Q2(1,i))+l*sin(new2_Q1(1,i));    
    
    
end




%% PLOTS 2B

t = linspace(0,T,dt);

new_Q1(dt+1) = [];
new_Q2(dt+1) = [];
new_dQ1(dt+1) = [];
new_dQ2(dt+1) = [];
new2_Q1(dt+1) = [];
new2_Q2(dt+1) = [];
new2_dQ1(dt+1) = [];
new2_dQ2(dt+1) = [];
xd(:,dt+1) = [];



% DESIRED AND ACTUAL ANGLE - AGAINST TIME
figure(4)
subplot(1,2,1)
plot(t,rad2deg(Q1), 'k--'); hold on; plot(t, rad2deg(new_Q1), 'b'); hold on; plot(t, rad2deg(new2_Q1), 'r');
title('DESIRED AND ACTUAL QL ANGLE');
legend('desired value', 'feedback', 'feedforward + feedback');
xlabel('Time [s]');
ylabel('Angle [\circ]')
subplot(1,2,2)
plot(t,rad2deg(Q2), 'k--'); hold on; plot(t, rad2deg(new_Q2), 'b'); hold on; plot(t, rad2deg(new2_Q2), 'r');
title('DESIRED AND ACTUAL QR ANGLE')
legend('desired value', 'feedback', 'feedforward + feedback')
xlabel('Time [s]');
ylabel('Angle [\circ]')

% DESIRED AND ACTUAL ENDPOINT POSITIONS X AND Y DIRECTIONS - AGAINST TIME
figure(5)
subplot(1,2,1)
plot(t,xd(1,:), 'k--'); hold on; plot(t, Xfb, 'b'); hold on; plot(t, Xff, 'r');
title('DESIRED AND ACTUAL ENDPOINT X POSITION');
legend('desired value', 'feedback', 'feedforward + feedback');
xlabel('Time [s]');
ylabel('Position [m]')
subplot(1,2,2)
plot(t,xd(2,:), 'k--'); hold on; plot(t, Yfb, 'b'); hold on; plot(t, Yff, 'r');
title('DESIRED AND ACTUAL ENDPOINT Y POSITION')
legend('desired value', 'feedback', 'feedforward + feedback')
xlabel('Time [s]');
ylabel('Position [m]')


% DESIRED AND ACTUAL ENDPOINT TRAJECTORIES - IN THE X-Y PLANE
figure(6)
plot(xd(1,:),xd(2,:), 'k--'); hold on; plot(Xfb, Yfb, 'b'); hold on; plot(Xff, Yff, 'r');
title('DESIRED AND ACTUAL ENDPOINT TRAJECTORIES');
legend('desired value', 'feedback', 'feedforward + feedback');
xlabel('x axis [m]');
ylabel('y axis [m]')




