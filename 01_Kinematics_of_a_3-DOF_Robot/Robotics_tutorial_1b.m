%% Q2_A Jacobian

syms a c q1 q2 q3

X = a*cos(q1) + a*cos(q2) + c*cos(q3);
Y = a*sin(q1) + a*sin(q2) + c*sin(q3);
H = [X; Y];

J11 = jacobian(H(1),q1);
J12 = jacobian(H(1),q2);
J13 = jacobian(H(1),q3);

J21 = jacobian(H(2),q1);
J22 = jacobian(H(2),q2);
J23 = jacobian(H(2),q3);

J = [J11 J12 J13; J21 J22 J23];

% output:
% J = 
% [ -a*sin(q1), -a*sin(q2), -c*sin(q3)]
% [  a*cos(q1),  a*cos(q2),  c*cos(q3)]