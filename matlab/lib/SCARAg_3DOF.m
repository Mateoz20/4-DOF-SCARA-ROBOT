function Gq = SCARAg_3DOF(Q,L)
% Gravity term for planar vertical 3R SCARA robot
% Using extended Jacobian and weight vector

g = 9.81;

% unpack masses
m1 = L(7);
m2 = L(8);
m3 = L(9);

% extended jacobian
Je = SCARAjacdin_3DOF(Q,L);

% weight vector acting on CoM coordinates in Se
Wg = zeros(10,1);

% y_g2 (rows 3,4)
Wg(4) = -m2 * g;

% y_g1 (rows 6,7)
Wg(7) = -m1 * g;

% y_g3 (rows 9,10)
Wg(10) = -m3 * g;

% joint gravity torques
Gq = Je' * Wg;
end
