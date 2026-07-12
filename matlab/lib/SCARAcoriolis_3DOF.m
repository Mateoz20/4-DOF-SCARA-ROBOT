function Cq = SCARAcoriolis_3DOF(Q, Qp, L)
% Coriolis and centrifugal term: C(q,qp)*qp
% Using extended-kinematic method:
%
%   Cq = Je' * W * Jep * Qp
%
% where:
%   Je  = extended Jacobian (10x3)
%   Jep = time derivative (10x3)
%   W   = mass-inertia matrix of the extended state (10x10)

%% ---- unpack masses and inertias ----
m1 = L(7);   m2 = L(8);   m3 = L(9);
I1 = L(10);  I2 = L(11);  I3 = L(12);

%% ---- Build W matrix (same as in SCARAM_3DOF) ----
W = zeros(10,10);

% masses at CoM coordinates
W(6,6)   = m1;    % x_g1
W(7,7)   = m1;    % y_g1
W(3,3)   = m2;    % x_g2
W(4,4)   = m2;    % y_g2
W(9,9)   = m3;    % x_g3
W(10,10) = m3;    % y_g3

% rotational inertia of the end effector orientation Se(5)
W(5,5) = I1 + I2 + I3;

% inertia of link 1 for Se(8)
W(8,8) = I1;

%% ---- Compute extended Jacobian and its time derivative ----
Je  = SCARAjacdin_3DOF(Q,L);
Jep = SCARAjacPdin_3DOF(Q,Qp,L);

%% ---- Coriolis / centrifugal term ----
Cq = Je' * W * (Jep * Qp);

end
