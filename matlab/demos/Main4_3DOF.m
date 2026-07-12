% main4_3DOF:
clc; clear all; close all;

l1 = 240; 
l2 = 252.5; 
l3 = 120;             
L  = [l1; l2; l3]';  % Length of the links

Si = [300; 300];     % TCP initial position (x,y)
Sf = [350; 400];     % TCP final position (x,y)
dS = Sf - Si;

Six = Si(1); dSx = dS(1);
Siy = Si(2); dSy = dS(2);

Qi = SCARAinv_3DOF(Si, L, -1);
Qf = SCARAinv_3DOF(Sf, L, -1);

Phi = 0;              % fixed orientation of the end-effector

T   = 9;
t1x = 3; t2x = 6; t3x = T;
t1y = 3; t2y = 6; t3y = T;

f1 = figure;
f1.WindowStyle = 'normal';
ll1 = line('XData', [0 0 0 0], ...
           'YData', [0 0 0 0], ...
           'linestyle', '-','linewidth',2,'color','b', ...
           'marker','o','markersize',6,'markerfacecolor','k');

i = 1;

for t = 0:0.1:T
    resx = Sshape_3DOF(t,Six,dSx,t1x,t2x,t3x);
    resy = Sshape_3DOF(t,Siy,dSy,t1y,t2y,t3y);
    
    S = [resx.pos; resy.pos];    % desired position (x,y)
    
    % inverse kinematic 3DOF
    Q = SCARAinv_3DOF(S,Phi,L,1); 

    % plot configuration
    PlotScara_3DOF(Q,L,'r',f1,ll1);

    % space of the task: [x; y; Phi]
    Sp  = [resx.vel;  resy.vel;  0];   % Phi costante -> vel 0
    Spp = [resx.acc;  resy.acc;  0];   % Phi costante -> acc 0

    J  = SCARAjac_3DOF(Q,L);
    Qp = inv(J)*Sp; 

    Jp  = SCARAjacP_3DOF(Q,Qp,L);
    Qpp = inv(J)*(Spp - Jp*Qp);

    time(i) = t;
    px(i) = resx.pos; vx(i) = resx.vel; ax(i) = resx.acc;
    py(i) = resy.pos; vy(i) = resy.vel; ay(i) = resy.acc;

    q1(i)   = Q(1);   q1p(i)   = Qp(1);   q1pp(i)   = Qpp(1);
    q2(i)   = Q(2);   q2p(i)   = Qp(2);   q2pp(i)   = Qpp(2);
    q3(i)   = Q(3);   q3p(i)   = Qp(3);   q3pp(i)   = Qpp(3);

    i = i+1;
end

PlotAreaSCARA_3DOF(L,f1);

figure(f1); hold on; plot(px,py,'--',LineWidth=2, Color='k'); hold off;

figure;
subplot(3,2,1); plot(time,px); grid; ylabel('px [mm]'); xlabel('time [s]');
subplot(3,2,2); plot(time,py); grid; ylabel('py [mm]'); xlabel('time [s]');
subplot(3,2,3); plot(time,vx); grid; ylabel('vx [mm/s]'); xlabel('time [s]');
subplot(3,2,4); plot(time,vy); grid; ylabel('vy [mm/s]'); xlabel('time [s]');
subplot(3,2,5); plot(time,ax); grid; ylabel('acc_x [mm/s^2]'); xlabel('time [s]');
subplot(3,2,6); plot(time,ay); grid; ylabel('acc_y [mm/s^2]'); xlabel('time [s]');


figure;
subplot(3,3,1); plot(time,q1); grid; ylabel('q1 [rad]'); xlabel('time [s]');
subplot(3,3,2); plot(time,q2); grid; ylabel('q2 [rad]'); xlabel('time [s]');
subplot(3,3,3); plot(time,q3); grid; ylabel('q3 [rad]'); xlabel('time [s]');
subplot(3,3,4); plot(time,q1p); grid; ylabel('q1p [rad/s]'); xlabel('time [s]');
subplot(3,3,5); plot(time,q2p); grid; ylabel('q2p [rad/s]'); xlabel('time [s]');
subplot(3,3,6); plot(time,q3p); grid; ylabel('q3p [rad/s]'); xlabel('time [s]');
subplot(3,3,7); plot(time,q1pp); grid; ylabel('q1pp [rad/s^2]'); xlabel('time [s]');
subplot(3,3,8); plot(time,q2pp); grid; ylabel('q2pp [rad/s^2]'); xlabel('time [s]');
subplot(3,3,9); plot(time,q3pp); grid; ylabel('q3pp [rad/s^2]'); xlabel('time [s]');

