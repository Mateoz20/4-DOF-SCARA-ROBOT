% %% ========================================================================
% %   SCARA 4-DOF – TRAJECTORY #2 (ADE) – CUBIC SPLINE (BOOK FORMULAS)
% %   Cinematica completa + Dinamica (3DOF) + Export per Simscape
% % ========================================================================
% clc; clear; close all;
% 
% %% -----------------------------
% %   ROBOT PARAMETERS
% % -----------------------------
% l1 = 0.24;
% l2 = 0.2525;
% l3 = 0.12;
% 
% g1 = 0.119;
% g2 = 0.063;
% g3 = 0.03;
% 
% m1 = 1.62396;
% m2 = 2.12630;
% m3 = 0.00832;
% 
% I1 = 10911411.81e-9;
% I2 = 14244066.35e-9;
% I3 = 12179.49e-9;
% 
% L = [l1;l2;l3; g1;g2;g3; m1;m2;m3; I1;I2;I3];
% 
% %% ------------------------------------------------------------------------
% %   INPUT ADE – 4 JOINT CONFIGURATIONS (degrees → radians)
% % ------------------------------------------------------------------------
% Q_deg = [ ...
%      0-90        0-90      13.15-90    15.17-90;     % q1
%    -12.56-180  -36.41-180   -43.51-180   -74.73-180;     % q2
%    -41.24+90  -17.85+90     6.85+90    60.09+90;     % q3
%      0        0        0        0];      % q4
% 
% Qk = deg2rad(Q_deg);     % 4 × 4
% 
% %% ------------------------------------------------------------------------
% %   TIME SETTINGS
% % ------------------------------------------------------------------------
% Ttot = 6;
% dt   = 0.01;
% tt   = 0:dt:Ttot;
% N    = length(tt);
% 
% tk = linspace(0, Ttot, 4);     % times corresponding to A-D-E-...
% 
% %% ------------------------------------------------------------------------
% %   SPLINE CUBICA (BOOK) FOR ALL 4 JOINTS
% % ------------------------------------------------------------------------
% Q   = zeros(4,N);
% Qp  = zeros(4,N);
% Qpp = zeros(4,N);
% 
% for j = 1:4
%     [Q(j,:), Qp(j,:), Qpp(j,:)] = SplineCubica(tk, Qk(j,:), tt);
% end
% 
% %% ------------------------------------------------------------------------
% %   KINEMATICS – TCP PATH (just for plot)
% % ------------------------------------------------------------------------
% TCP = zeros(2,N);
% 
% for i = 1:N
%     q1 = Q(1,i); q2 = Q(2,i); q3 = Q(3,i);
%     TCP(:,i) = [
%         l1*cos(q1) + l2*cos(q1+q2) + l3*cos(q1+q2+q3);
%         l1*sin(q1) + l2*sin(q1+q2) + l3*sin(q1+q2+q3)
%     ];
% end
% 
% %% ------------------------------------------------------------------------
% %   DYNAMICS – ONLY FIRST 3 DOF
% % ------------------------------------------------------------------------
% Fext = [0; -5; 0];   % 5 N down on end-effector
% 
% Fq_in = zeros(3,N);
% Fq_co = zeros(3,N);
% Fq_gr = zeros(3,N);
% Fq_ex = zeros(3,N);
% Fq    = zeros(3,N);
% 
% for i = 1:N
% 
%     q   = Q(1:3,i);
%     qp  = Qp(1:3,i);
%     qpp = Qpp(1:3,i);
% 
%     Mq = SCARAM_3DOF(q,L);
%     Cq = SCARAcoriolis_3DOF(q, qp, L);
%     Gq = SCARAg_3DOF(q, L);
%     J  = SCARAjac_3DOF(q, L);
% 
%     tau_ext = J' * Fext;
% 
%     Fq_in(:,i) = Mq * qpp;
%     Fq_co(:,i) = Cq;
%     Fq_gr(:,i) = Gq;
%     Fq_ex(:,i) = tau_ext;
% 
%     Fq(:,i) = Fq_in(:,i) + Fq_co(:,i) + Fq_gr(:,i) + Fq_ex(:,i);
% end
% 
% %% ------------------------------------------------------------------------
% %   PLOTS – JOINT TRAJECTORIES
% % ------------------------------------------------------------------------
% figure;
% subplot(4,1,1); plot(tt,Q(1,:)); grid on; ylabel('q1 [rad]');
% subplot(4,1,2); plot(tt,Q(2,:)); grid on; ylabel('q2 [rad]');
% subplot(4,1,3); plot(tt,Q(3,:)); grid on; ylabel('q3 [rad]');
% subplot(4,1,4); plot(tt,Q(4,:)); grid on; ylabel('q4 [rad]');
% xlabel('time'); sgtitle('ADE – Joint Positions');
% 
% figure;
% subplot(4,1,1); plot(tt,Qp(1,:)); grid on; ylabel('q1 dot');
% subplot(4,1,2); plot(tt,Qp(2,:)); grid on; ylabel('q2 dot');
% subplot(4,1,3); plot(tt,Qp(3,:)); grid on; ylabel('q3 dot');
% subplot(4,1,4); plot(tt,Qp(4,:)); grid on; ylabel('q4 dot');
% xlabel('time'); sgtitle('ADE – Joint Velocities');
% 
% figure;
% subplot(4,1,1); plot(tt,Qpp(1,:)); grid on; ylabel('q1 ddot');
% subplot(4,1,2); plot(tt,Qpp(2,:)); grid on; ylabel('q2 ddot');
% subplot(4,1,3); plot(tt,Qpp(3,:)); grid on; ylabel('q3 ddot');
% subplot(4,1,4); plot(tt,Qpp(4,:)); grid on; ylabel('q4 ddot');
% xlabel('time'); sgtitle('ADE – Joint Accelerations');
% 
% %% ------------------------------------------------------------------------
% %   PLOT TORQUES (3 DOF ONLY)
% % ------------------------------------------------------------------------
% figure;
% subplot(3,1,1); plot(tt, Fq(1,:)); grid on; ylabel('\tau_1 [Nm]');
% subplot(3,1,2); plot(tt, Fq(2,:)); grid on; ylabel('\tau_2 [Nm]');
% subplot(3,1,3); plot(tt, Fq(3,:)); grid on; ylabel('\tau_3 [Nm]');
% xlabel('time'); sgtitle('ADE – Joint Torques');
% 
% %% ------------------------------------------------------------------------
% %   EXPORT FOR SIMSCAPE (ADE)
% % ------------------------------------------------------------------------
% offset_q1 = deg2rad(90);
% offset_q2 = deg2rad(180);
% offset_q3 = 0;
% offset_q4 = 0;
% 
% q1_sig_ADE = struct('time',tt(:), 'signals',struct('values',Q(1,:)'+offset_q1,'dimensions',1));
% q2_sig_ADE = struct('time',tt(:), 'signals',struct('values',Q(2,:)'+offset_q2,'dimensions',1));
% q3_sig_ADE = struct('time',tt(:), 'signals',struct('values',Q(3,:)'+offset_q3,'dimensions',1));
% q4_sig_ADE = struct('time',tt(:), 'signals',struct('values',Q(4,:)'+offset_q4,'dimensions',1));
% 
% save('ADE_traj.mat','q1_sig_ADE','q2_sig_ADE','q3_sig_ADE','q4_sig_ADE');
% 
% disp('Saved: ADE_traj.mat');
% 
% %% ------------------------------------------------------------------------
% %   ANIMAZIONE 3D REALISTICA – braccio con spessore
% % ------------------------------------------------------------------------
% figure; hold on; grid on; axis equal;
% title('SCARA ADE – 4DOF 3D Animation (Thick Links)');
% xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
% 
% app = (l1 + l2 + l3)*1.2;
% xlim([-app app]); ylim([-app app]); zlim([-0.15 0.25]);
% 
% view(45,25);   % camera angled → essential for 3D perception
% 
% % thickness of links in the vertical direction
% z1_off = 0.05;
% z2_off = 0.10;
% z3_off = 0.15;
% 
% h = plot3([0],[0],[0],'-o','LineWidth',4,...
%           'MarkerFaceColor','k','MarkerSize',8);
% 
% for i = 1:10:N
% 
%     q1 = Q(1,i);
%     q2 = Q(2,i);
%     q3 = Q(3,i);
%     q4 = Q(4,i);
% 
%     % ---- PLANAR POSITIONS (XY plane) ----
%     P1_local = [ l1*cos(q1);                 l1*sin(q1);                 z1_off];
%     P2_local = [ l1*cos(q1)+l2*cos(q1+q2);   l1*sin(q1)+l2*sin(q1+q2);   z2_off];
%     P3_local = [ P2_local(1)+l3*cos(q1+q2+q3);
%                  P2_local(2)+l3*sin(q1+q2+q3);
%                  z3_off ];
% 
%     % ---- ROTATION MATRIX AROUND Z (q4) ----
%     Rz = [cos(q4) -sin(q4) 0;
%           sin(q4)  cos(q4) 0;
%                0        0  1];
% 
%     % rotate entire arm around vertical axis
%     P1 = Rz * P1_local;
%     P2 = Rz * P2_local;
%     P3 = Rz * P3_local;
% 
%     % update animation
%     set(h,'XData',[0 P1(1) P2(1) P3(1)], ...
%           'YData',[0 P1(2) P2(2) P3(2)], ...
%           'ZData',[0 P1(3) P2(3) P3(3)]);
% 
%     drawnow;
% end
% 
% 






%% ========================================================================
%   SCARA 4DOF – TRAIETTORIA ADE (Spline Cubica MATLAB)
%   Cinematica + Dinamica (3DOF) + Simscape export + Animazione 3D
% ========================================================================
clc; clear; close all;

%% -----------------------------
%   ROBOT PARAMETERS
% -----------------------------
l1 = 0.24;  l2 = 0.2525;  l3 = 0.12;

g1 = 0.119; g2 = 0.063; g3 = 0.03;

m1 = 1.62396; m2 = 2.12630; m3 = 0.00832;

I1 = 10911411.81e-9;
I2 = 14244066.35e-9;
I3 = 12179.49e-9;

L = [l1;l2;l3; g1;g2;g3; m1;m2;m3; I1;I2;I3];

%% ------------------------------------------------------------------------
%   INPUT ADE – 4 JOINT CONFIGURATIONS (degrees → radians)
% ------------------------------------------------------------------------
Q_deg = [ ...
     0          0       9.92       11.89;     % q1
     3.29    -25.13     -33.95     -55.73;     % q2
   -33.06    -29        -25.7+29   -15.21+25.7+29 ;     % q3
    -5.23    -5.23      -5.23       -5.23 ];     % q4 (always flat)

Qk = deg2rad(Q_deg);   % 4×4 matrix

%% ------------------------------------------------------------------------
%   TIME SETTINGS
% ------------------------------------------------------------------------
Ttot = 7.5;          % total movement duration
dt   = 0.01;
tt   = 0:dt:Ttot;   % full time vector
N    = length(tt);

tk = linspace(0, Ttot, 4);   % time instants for the 4 points (A-D-E)

%% ------------------------------------------------------------------------
%   SPLINE MATLAB FOR ALL 4 JOINTS
% ------------------------------------------------------------------------
Q   = zeros(4,N);
Qp  = zeros(4,N);
Qpp = zeros(4,N);

for j = 1:4
    % Build MATLAB cubic spline (natural)
    pp = spline(tk, Qk(j,:));
    
    % Evaluate position, velocity, acceleration
    Q(j,:)   = ppval(pp, tt);
    Qp(j,:)  = ppval(fnder(pp,1), tt);
    Qpp(j,:) = ppval(fnder(pp,2), tt);
end

%% ------------------------------------------------------------------------
%   KINEMATICS – TCP PATH (for plotting)
% ------------------------------------------------------------------------
TCP = zeros(2,N);

for i = 1:N
    q1 = Q(1,i); q2 = Q(2,i); q3 = Q(3,i);

    TCP(:,i) = [
        l1*cos(q1) + l2*cos(q1+q2) + l3*cos(q1+q2+q3);
        l1*sin(q1) + l2*sin(q1+q2) + l3*sin(q1+q2+q3)
    ];
end

%% ------------------------------------------------------------------------
%   DYNAMICS – ONLY FIRST 3 DOF
% ------------------------------------------------------------------------
Fext = [0; 0; 0];   % 5 N downward force

Fq_in = zeros(3,N);
Fq_co = zeros(3,N);
Fq_gr = zeros(3,N);
Fq_ex = zeros(3,N);
Fq    = zeros(3,N);

for i = 1:N

    q   = Q(1:3,i);
    qp  = Qp(1:3,i);
    qpp = Qpp(1:3,i);

    Mq = SCARAM_3DOF(q,L);
    Cq = SCARAcoriolis_3DOF(q, qp, L);
    Gq = SCARAg_3DOF(q, L);
    J  = SCARAjac_3DOF(q, L);

    tau_ext = J' * Fext;

    Fq_in(:,i) = Mq * qpp;
    Fq_co(:,i) = Cq;
    Fq_gr(:,i) = Gq;
    Fq_ex(:,i) = tau_ext;

    Fq(:,i) = Fq_in(:,i) + Fq_co(:,i) + Fq_gr(:,i) + Fq_ex(:,i);
end

%% ------------------------------------------------------------------------
%   PLOTS – Q, Qp, Qpp
% ------------------------------------------------------------------------
figure;
subplot(4,1,1); plot(tt,Q(1,:)); grid on; ylabel('q1 [rad]');
subplot(4,1,2); plot(tt,Q(2,:)); grid on; ylabel('q2 [rad]');
subplot(4,1,3); plot(tt,Q(3,:)); grid on; ylabel('q3 [rad]');
subplot(4,1,4); plot(tt,Q(4,:)); grid on; ylabel('q4 [rad]');
xlabel('time'); sgtitle('ADE – Joint Positions');

figure;
subplot(4,1,1); plot(tt,Qp(1,:)); grid on; ylabel('q1 dot');
subplot(4,1,2); plot(tt,Qp(2,:)); grid on; ylabel('q2 dot');
subplot(4,1,3); plot(tt,Qp(3,:)); grid on; ylabel('q3 dot');
subplot(4,1,4); plot(tt,Qp(4,:)); grid on; ylabel('q4 dot');
xlabel('time'); sgtitle('ADE – Joint Velocities');

figure;
subplot(4,1,1); plot(tt,Qpp(1,:)); grid on; ylabel('q1 ddot');
subplot(4,1,2); plot(tt,Qpp(2,:)); grid on; ylabel('q2 ddot');
subplot(4,1,3); plot(tt,Qpp(3,:)); grid on; ylabel('q3 ddot');
subplot(4,1,4); plot(tt,Qpp(4,:)); grid on; ylabel('q4 ddot');
xlabel('time'); sgtitle('ADE – Joint Accelerations');

%% ------------------------------------------------------------------------
%   PLOT TORQUES (3 DOF ONLY)
% ------------------------------------------------------------------------
figure;
subplot(3,1,1); plot(tt,Fq(1,:)); grid on; ylabel('\tau_1 [Nm]');
subplot(3,1,2); plot(tt,Fq(2,:)); grid on; ylabel('\tau_2 [Nm]');
subplot(3,1,3); plot(tt,Fq(3,:)); grid on; ylabel('\tau_3 [Nm]');
xlabel('time'); sgtitle('ADE – Joint Torques');

%% ------------------------------------------------------------------------
%   EXPORT FOR SIMSCAPE (ADE)
% ------------------------------------------------------------------------
offset_q1 = 0*deg2rad(90);
offset_q2 = 0*deg2rad(180);
offset_q3 = 0;
offset_q4 = 0;

q1_sig_ADE = struct('time',tt(:), 'signals',struct('values',Q(1,:)'+offset_q1,'dimensions',1));
q2_sig_ADE = struct('time',tt(:), 'signals',struct('values',Q(2,:)'+offset_q2,'dimensions',1));
q3_sig_ADE = struct('time',tt(:), 'signals',struct('values',Q(3,:)'+offset_q3,'dimensions',1));
q4_sig_ADE = struct('time',tt(:), 'signals',struct('values',Q(4,:)'+offset_q4,'dimensions',1));

save('ADE_traj.mat','q1_sig_ADE','q2_sig_ADE','q3_sig_ADE','q4_sig_ADE');
disp('Saved ADE_traj.mat for Simscape.');


%% ------------------------------------------------------------------------
%   3D ANIMATION (with q4 rotation)
% ------------------------------------------------------------------------
figure; hold on; grid on; axis equal;
title('SCARA ADE – 4DOF 3D Animation');
xlabel('X'); ylabel('Y'); zlabel('Z');

app = (l1+l2+l3)*1.2;
xlim([-app app]); ylim([-app app]); zlim([-0.1 0.25]);
view(45,25);

h = plot3(0,0,0,'-o','LineWidth',2,'MarkerFaceColor','k','MarkerSize',6);

for i = 1:10:N
    q1 = Q(1,i); q2 = Q(2,i); q3 = Q(3,i); q4 = Q(4,i);

    % plan view (z = 0)
    P1 = [ l1*cos(q1);                l1*sin(q1);               0.05];
    P2 = [ l1*cos(q1)+l2*cos(q1+q2);
           l1*sin(q1)+l2*sin(q1+q2);  0.10];
    P3 = [ P2(1)+l3*cos(q1+q2+q3);
           P2(2)+l3*sin(q1+q2+q3);    0.15];

    % rotation around Z (q4)
    Rz = [cos(q4) -sin(q4) 0;
          sin(q4)  cos(q4) 0;
               0        0  1];

    P1 = Rz*P1; P2 = Rz*P2; P3 = Rz*P3;

    set(h,'XData',[0 P1(1) P2(1) P3(1)], ...
          'YData',[0 P1(2) P2(2) P3(2)], ...
          'ZData',[0 P1(3) P2(3) P3(3)]);
    drawnow;
end

