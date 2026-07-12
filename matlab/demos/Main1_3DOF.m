%  % TEST SCARA
%  % script to test the direct and inverse kinematic problem: SCARA robot
% 
% clc; 
% clear all;
% 
% %L=[l1; l2; l3]';
% l1 = 240; l2 = 252.5; l3=120 ; % Length of the links
% L=[l1; l2; l3]';
% S = [500,200];
% Phi = atan2(S(2), S(1)); % Calculate the angle for the TCP position
% 
% S=[400; 400]; % TCP position
% 
% Q1=SCARAinv_3DOF(S,Phi,L ,1); % first solution (beta >0)
% Q2=SCARAinv_3DOF(S,Phi,L ,-1); % second solution (beta <0)
% 
% S1=SCARAdir_3DOF(Q1 ,L); % check 1 sol
% S2=SCARAdir_3DOF(Q2 ,L); % check 2 sol
% 
% f1=figure;
% f1.WindowStyle='normal';
% ll1=line('XData', [0 0 0], ...
%  'YData', [0 0 0], ...
%  'linestyle', '-','linewidth',2,'color','b',...
%  'marker','o','markersize',6,'markerfacecolor','k');
% ll2=line('XData', [0 0 0], ...
%  'YData', [0 0 0], ...
%  'linestyle', '-','linewidth',2,'color','b',...
%  'marker','o','markersize',6,'markerfacecolor','k');
% 
% 
% PlotScara_3DOF(Q1,L,'r',f1,ll1) % draw the 1 sol. 
% PlotScara_3DOF(Q2,L,'b',f1,ll2) % draw the 2 sol.
% hold on
% PlotAreaSCARA_3DOF(L,f1); % draw the work area

% main01_3DOF:
% TEST SCARA 3DOF
% script to test the direct and inverse kinematic problem: SCARA robot 3DOF

clc;
clear all;

l1 = 240; 
l2 = 252.5; 
l3 = 120;           % Length of the links
L  = [l1; l2; l3]';

S   = [400; 400];   % TCP position (x,y)
Phi = 0;            % orientazione desiderata dell'end-effector

Q1 = SCARAinv_3DOF(S,Phi,L, 1);   % first solution (beta > 0)
Q2 = SCARAinv_3DOF(S,Phi,L,-1);   % second solution (beta < 0)

S1 = SCARAdir_3DOF(Q1,L);         % check 1st sol
S2 = SCARAdir_3DOF(Q2,L);         % check 2nd sol

f1 = figure;
f1.WindowStyle = 'normal';

ll1 = line('XData', [0 0 0 0], ...
           'YData', [0 0 0 0], ...
           'linestyle', '-','linewidth',2,'color','b', ...
           'marker','o','markersize',6,'markerfacecolor','k');
ll2 = line('XData', [0 0 0 0], ...
           'YData', [0 0 0 0], ...
           'linestyle', '-','linewidth',2,'color','b', ...
           'marker','o','markersize',6,'markerfacecolor','k');

PlotScara_3DOF(Q1,L,'r',f1,ll1); % draw the 1st solution 
PlotScara_3DOF(Q2,L,'b',f1,ll2); % draw the 2nd solution
hold on
PlotAreaSCARA_3DOF(L,f1);        % draw the work area
