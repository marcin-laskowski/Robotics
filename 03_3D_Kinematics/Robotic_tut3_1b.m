%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Robotic_tut3_1b
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% calculating partial derivatives of the Jacobian Matrix
J11 = jacobian(T(1,4),theta1);
J12 = jacobian(T(1,4),theta2);
J13 = jacobian(T(1,4),theta3);
J14 = jacobian(T(1,4),theta4);
J15 = jacobian(T(1,4),theta5);
J16 = jacobian(T(1,4),theta6);
J17 = jacobian(T(1,4),theta7);

J21 = jacobian(T(2,4),theta1);
J22 = jacobian(T(2,4),theta2);
J23 = jacobian(T(2,4),theta3);
J24 = jacobian(T(2,4),theta4);
J25 = jacobian(T(2,4),theta5);
J26 = jacobian(T(2,4),theta6);
J27 = jacobian(T(2,4),theta7);

J31 = jacobian(T(3,4),theta1);
J32 = jacobian(T(3,4),theta2);
J33 = jacobian(T(3,4),theta3);
J34 = jacobian(T(3,4),theta4);
J35 = jacobian(T(3,4),theta5);
J36 = jacobian(T(3,4),theta6);
J37 = jacobian(T(3,4),theta7);

% Jacobian Matrix
J = [J11 J12 J13 J14 J15 J16 J17; J21 J22 J23 J24 J25 J26 J27; J31 J32 J33 J34 J35 J36 J37];
J = simplify(J);

% substituting given values from the D-H table
subs(J,{a1,a2,a3,a4,a5,a6,a7},{0,0.069,0,0.069,0,0.01,0})
J = ans;
subs(J,{d1,d2,d3,d4,d5,d6,d7},{0.2703,0,0.3644,0,0.3743,0,0.2295})
J = ans;
subs(J,{alpha1,alpha2,alpha3,alpha4,alpha5,alpha6,alpha7},{0,-1.571,1.571,-1.571,1.571,-1.571,1.571})
J = ans;
