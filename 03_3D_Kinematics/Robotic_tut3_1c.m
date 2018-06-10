%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Robotic_tut3_1c
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% sampling step and time
Time = 4;
dt = 0.002;
t = 0:dt:Time;

% position
xx = 0.05*cos(0.5*pi*t);
xy = 0.05*sin(0.5*pi*t);
xz = zeros(1,length(t));
X = [xx; xy; xz];

% velocity
vx = 0.05*(-sin(0.5*pi*t))*(0.5*pi);
vy = 0.05*(cos(0.5*pi*t))*(0.5*pi);
vz = zeros(1,length(t));
V = [vx; vy; vz];

% angle ranges
S0 = [-1.7016, 1.7016];
S1 = [-2.147, 1.047];
E0 = [-3.0541, 3.0541];
E1 = [-0.05, 2.618];
W0 = [-3.059, 3.059];
W1 = [-1.5707, 2.094];
W2 = [-3.059, 3.059];

Angle_ranges = [S0; S1; E0; E1; W0; W1; W2];

% Initial Q (Q = [q1; q2; q3; q4; q5; q6; q7])
LowerBound = [Angle_ranges(:,1)];
Range = -Angle_ranges(:,1) + Angle_ranges(:,2);

Q = [];
Q = LowerBound + 0.5*Range;

% Inverse Kinematics of differential motion: dQ = pinv(J)*V

for i = 1:length(t) 
    
    loading = i
    J_temp = subs(J,{theta1,theta2,theta3,theta4,theta5,theta6,theta7},{Q(1,i),Q(2,i),Q(3,i),Q(4,i),Q(5,i),Q(6,i),Q(7,i)});
    JJ = double(J_temp);
    
    % VELOCITY - DESIRED dQ
    dQ(:,i) = pinv(JJ) * V(:,i);
    
    % POSITION - DESIRED Q
    Q(:,i+1) = Q(:,i) + dQ(:,i)*dt;
       
end



% Calculating the x, y and z for the end effector drawing circle
Txyz = [T(1,4); T(2,4); T(3,4)]

subs(Txyz,{a1,a2,a3,a4,a5,a6,a7},{0,0.069,0,0.069,0,0.01,0})
Txyz = ans;
subs(Txyz,{d1,d2,d3,d4,d5,d6,d7},{0.2703,0,0.3644,0,0.3743,0,0.2295})
Txyz = ans;
subs(Txyz,{alpha1,alpha2,alpha3,alpha4,alpha5,alpha6,alpha7},{0,-1.571,1.571,-1.571,1.571,-1.571,1.571})
Txyz = ans;

TT_NEW = [];

for i = 1:length(t)
    
    loading_XYZ = i
    Txyz_temp = subs(Txyz,{theta1,theta2,theta3,theta4,theta5,theta6,theta7},{Q(1,i),Q(2,i),Q(3,i),Q(4,i),Q(5,i),Q(6,i),Q(7,i)});
    TT = double(Txyz_temp);
    TT_NEW(:,i) = TT;
    
    x(i) = TT(1,1);
    y(i) = TT(2,1);
    z(i) = TT(3,1);
    
end
