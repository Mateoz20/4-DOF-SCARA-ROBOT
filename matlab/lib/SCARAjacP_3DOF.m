function Jp = SCARAjacP_3DOF(Q,Qp,L)
% Time derivative of the Jacobian matrix: SCARA robot 3DOF
l1 = L(1);
l2 = L(2);
l3 = L(3);

q1  = Q(1);  q2  = Q(2);  q3  = Q(3);
q1p = Qp(1); q2p = Qp(2); q3p = Qp(3);

Jp = zeros(3,3);

c1   = cos(q1);          s1   = sin(q1);
c12  = cos(q1+q2);       s12  = sin(q1+q2);
c123 = cos(q1+q2+q3);    s123 = sin(q1+q2+q3);

q12p  = q1p + q2p;
q123p = q1p + q2p + q3p;

% riga 1 (derivate delle componenti di x)
Jp(1,1) = -l1*c1*q1p - l2*c12*q12p - l3*c123*q123p;
Jp(1,2) =           - l2*c12*q12p - l3*c123*q123p;
Jp(1,3) =                         - l3*c123*q123p;

% riga 2 (derivate delle componenti di y)
Jp(2,1) = -l1*s1*q1p - l2*s12*q12p - l3*s123*q123p;
Jp(2,2) =           - l2*s12*q12p - l3*s123*q123p;
Jp(2,3) =                         - l3*s123*q123p;

% riga 3 (Phi = q1+q2+q3 -> derivate costanti)
Jp(3,1) = 0;
Jp(3,2) = 0;
Jp(3,3) = 0;
end