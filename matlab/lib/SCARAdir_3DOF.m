% function [S,Phi]=SCARAdir_3DOF(Q,L)
% % direct kinematics: SCARA robot
% l1=L(1);
% l2=L(2);
% l3=L(3);
% 
% S= zeros(2,1);
% 
% S(1)=l1*cos(Q(1))+l2*cos(Q(1)+Q(2))+l3*cos(Q(1)+Q(2)+Q(3));
% S(2)= l1*sin(Q(1))+ l2*sin(Q(1)+Q(2))+l3*sin(Q(1)+Q(2)+Q(3));
% Phi= Q(1)+Q(2)+Q(3);
% 
% end

function S = SCARAdir_3DOF(Q,L)
% direct kinematics: SCARA robot 3DOF
l1 = L(1);
l2 = L(2);
l3 = L(3);

q1 = Q(1);
q2 = Q(2);
q3 = Q(3);

S = zeros(3,1);
S(1) = l1*cos(q1) + l2*cos(q1+q2) + l3*cos(q1+q2+q3); % x
S(2) = l1*sin(q1) + l2*sin(q1+q2) + l3*sin(q1+q2+q3); % y
S(3) = q1 + q2 + q3;                                   % Phi (orientazione end-effector)
end