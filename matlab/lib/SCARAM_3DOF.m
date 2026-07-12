function M = SCARAM_3DOF(Q,L)
% Mass matrix of a planar 3R SCARA robot
% M = Je' * W * Je

% unpack parameters
l1 = L(1);  l2 = L(2);  l3 = L(3);
g1 = L(4);  g2 = L(5);  g3 = L(6);

m1 = L(7);  m2 = L(8);  m3 = L(9);
I1 = L(10); I2 = L(11); I3 = L(12);

% extended jacobian
Je = SCARAjacdin_3DOF(Q,L);

% W matrix (10x10)
W = zeros(10,10);

% masses on CoM coordinates
W(6,6)   = m1;   % x_g1
W(7,7)   = m1;   % y_g1

W(3,3)   = m2;   % x_g2
W(4,4)   = m2;   % y_g2

W(9,9)   = m3;   % x_g3
W(10,10) = m3;   % y_g3

% rotational inertia on theta (Se(5))
W(5,5) = I1 + I2 + I3;

% rotational inertia of link 1 (alpha = q1)
W(8,8) = I1;

% mass matrix
M = Je' * W * Je;

end
