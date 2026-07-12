% function Q = SCARAinv_3DOF(S,Phi,L,sol)
% % inverse kinematic: SCARA robot 
% Q = zeros(3,1);
% x=S(1); y=S(2);
% %Phi* = Phi;
% l1=L(1); 
% l2=L(2); 
% l3=L(3);
% 
% %inverse kinematic
% xw = x-l3*cos(Phi);
% yw = y-l3*sin(Phi);
% % beta = acos((xw^2+ yw^2-l1^2 -l2^2)/(2*l1*l2));
% % if (sol>0)
% %     Q(2)=beta;
% % else 
% %     Q(2)= -beta;
% % end
% % 
% % Q(1)= atan2(yw,xw)- atan2(l2*sin(Q(2)), l1+l2*cos(Q(2));
% % Q(3) = Phi - Q(1) - Q(2);
% % end
% 
% %------------------------------------------------
% 
% % clamp for numerical safety
% c = (xw^2 + yw^2 - l1^2 - l2^2) / (2*l1*l2);
% c = max(-1, min(1, c));
% 
% beta = acos(c);
% if sol > 0
%     Q(2) = beta;
% else
%     Q(2) = -beta;
% end
% 
% Q(1) = atan2(yw, xw) - atan2(l2*sin(Q(2)), l1 + l2*cos(Q(2)));
% Q(3) = Phi - Q(1) - Q(2);
% end


function Q = SCARAinv_3DOF(S,Phi,L,sol)
% inverse kinematic: SCARA robot 3DOF
Q = zeros(3,1);
x = S(1);
y = S(2);

l1 = L(1);
l2 = L(2);
l3 = L(3);

% posizione del polso (giunto tra l2 e l3)
xw = x - l3*cos(Phi);
yw = y - l3*sin(Phi);

% inverse kinematic 2R sul "polso"
cosq2 = (xw^2 + yw^2 - l1^2 - l2^2) / (2*l1*l2);
cosq2 = max(min(cosq2, 1), -1);   % clamp in [-1,1]

beta = acos(cosq2);

if (sol > 0)
    Q(2) = beta;   % soluzione "gomito alto"
else 
    Q(2) = -beta;  % soluzione "gomito basso"
end

Q(1) = atan2(yw,xw) - atan2(l2*sin(Q(2)), l1 + l2*cos(Q(2)));

% terzo giunto dalla Phi desiderata
Q(3) = Phi - Q(1) - Q(2);
end

