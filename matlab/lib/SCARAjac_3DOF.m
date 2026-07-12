function J = SCARAjac_3DOF(Q,L)
% jacobian matrix: SCARA robot 3DOF
l1 = L(1);
l2 = L(2);
l3 = L(3);

q1 = Q(1);
q2 = Q(2);
q3 = Q(3);

J = zeros(3,3);

c1   = cos(q1);          s1   = sin(q1);
c12  = cos(q1+q2);       s12  = sin(q1+q2);
c123 = cos(q1+q2+q3);    s123 = sin(q1+q2+q3);

% righe: [x; y; Phi]  colonne: [q1 q2 q3]
J(1,1) = -l1*s1 - l2*s12 - l3*s123;
J(1,2) =        - l2*s12 - l3*s123;
J(1,3) =                   - l3*s123;

J(2,1) =  l1*c1 + l2*c12 + l3*c123;
J(2,2) =          l2*c12 + l3*c123;
J(2,3) =                    l3*c123;

J(3,1) = 1;
J(3,2) = 1;
J(3,3) = 1;
end