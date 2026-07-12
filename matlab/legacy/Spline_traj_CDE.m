% %% =============================================================
% %   SCARA 3DOF – Second trajectory using CUBIC SPLINES
% %   Joint-space interpolation through 4 given configurations
% %   q(t), qdot(t), qddot(t) + full dynamics + export for Simscape
% % =============================================================
% clc; clear; close all;
% 
% %% -------------------------------------------------------------
% % ROBOT PARAMETERS (same as before)
% %% -------------------------------------------------------------
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
% L = [l1; l2; l3; g1; g2; g3; m1; m2; m3; I1; I2; I3];
% 
% %% -------------------------------------------------------------
% %  FOUR JOINT CONFIGURATIONS (DEGREES -> RADIANS)
% %  q1, q2, q3 active; q4 currently zero in all points
% %% -------------------------------------------------------------
% % Q2_deg = [ ...
% %      0        0      13.15    15.17;   % q1
% %    -12.56  -36.41   -43.51   -74.73;   % q2
% %    -41.24  -17.85     6.85    60.09;   % q3
% %      0        0        0        0];    % q4 (base, here all zeros)
% 
% Q2_deg = [ ...
%      0          0        9.92          11.89;   % q1
%    -3.29      25.13      -33.95        -55.73;   % q2
%    -33.06     -29        -25.7         -15.21;   % q3
%    -5.23     -5.23      -5.23         -5.23];    % q4
% 
% 
% Qk_rad = deg2rad(Q2_deg);    % 4x4 in radians
% 
% 
% % Only first 3 joints are involved in planar dynamics
% Qk = Qk_rad(1:3,:);          % 3x4
% 
% %% -------------------------------------------------------------
% %  TIME GRID FOR SPLINES
% %% -------------------------------------------------------------
% Ttot = 6;            % total duration [s]
% dt   = 0.01;         % time step
% tt   = 0:dt:Ttot;    % time vector
% n    = length(tt);
% 
% % key times for the 4 knot points (uniform spacing)
% tk = linspace(0, Ttot, 4);   % [0, 2, 4, 6] if Ttot=6
% 
% %% -------------------------------------------------------------
% %  CUBIC SPLINE TRAJECTORY IN JOINT SPACE
% %  q(t), qdot(t), qddot(t) via spline + numerical gradient
% %% -------------------------------------------------------------
% Q   = zeros(3,n);
% Qp  = zeros(3,n);
% Qpp = zeros(3,n);
% 
% for j = 1:3   % for each joint
%     % spline interpolation through the 4 key configurations
%     qj = spline(tk, Qk(j,:), tt);   % position
% 
%     % numerical derivatives for velocity and acceleration
%     qjp  = gradient(qj, dt);       % velocity
%     qjpp = gradient(qjp, dt);      % acceleration
% 
%     Q(j,:)   = qj;
%     Qp(j,:)  = qjp;
%     Qpp(j,:) = qjpp;
% end
% 
% %% -------------------------------------------------------------
% %  TCP TRAJECTORY FOR PLOTTING (planar, 3-DOF)
% %% -------------------------------------------------------------
% TCP = zeros(2,n);
% for i = 1:n
%     q1 = Q(1,i); q2 = Q(2,i); q3 = Q(3,i);
%     TCP(:,i) = [
%         l1*cos(q1) + l2*cos(q1+q2) + l3*cos(q1+q2+q3);
%         l1*sin(q1) + l2*sin(q1+q2) + l3*sin(q1+q2+q3)
%     ];
% end
% 
% %% -------------------------------------------------------------
% %  DYNAMICS + EXTERNAL FORCE (5 N downward on TCP)
% %% -------------------------------------------------------------
% Fext = [0; -5; 0];   % external force in (x,y,phi) space
% 
% Fq_inertia  = zeros(3,n);
% Fq_coriolis = zeros(3,n);
% Fq_gravity  = zeros(3,n);
% Fq_ext      = zeros(3,n);
% Fq_total    = zeros(3,n);
% 
% for i = 1:n
%     q   = Q(:,i);
%     qp  = Qp(:,i);
%     qpp = Qpp(:,i);
% 
%     M  = SCARAM_3DOF(q,L);
%     Cq = SCARAcoriolis_3DOF(q,qp,L);
%     Gq = SCARAg_3DOF(q,L);
%     J  = SCARAjac_3DOF(q,L);
% 
%     tau_ext = J' * Fext;
% 
%     Fq_inertia(:,i)  = M*qpp;
%     Fq_coriolis(:,i) = Cq;
%     Fq_gravity(:,i)  = Gq;
%     Fq_ext(:,i)      = tau_ext;
% 
%     Fq_total(:,i) = Fq_inertia(:,i) + Fq_coriolis(:,i) + Fq_gravity(:,i) + Fq_ext(:,i);
% end
% 
% %% -------------------------------------------------------------
% %  PLOTS: JOINT POSITION, VELOCITY, ACCELERATION
% %% -------------------------------------------------------------
% figure;
% subplot(3,1,1); plot(tt, Q(1,:), 'LineWidth', 1.4); grid on;
% ylabel('q1 [rad]');
% subplot(3,1,2); plot(tt, Q(2,:), 'LineWidth', 1.4); grid on;
% ylabel('q2 [rad]');
% subplot(3,1,3); plot(tt, Q(3,:), 'LineWidth', 1.4); grid on;
% ylabel('q3 [rad]');
% xlabel('time [s]');
% sgtitle('Cubic spline trajectory – Joint Positions');
% 
% figure;
% subplot(3,1,1); plot(tt, Qp(1,:), 'LineWidth', 1.4); grid on;
% ylabel('q1̇ [rad/s]');
% subplot(3,1,2); plot(tt, Qp(2,:), 'LineWidth', 1.4); grid on;
% ylabel('q2̇ [rad/s]');
% subplot(3,1,3); plot(tt, Qp(3,:), 'LineWidth', 1.4); grid on;
% ylabel('q3̇ [rad/s]');
% xlabel('time [s]');
% sgtitle('Cubic spline trajectory – Joint Velocities');
% 
% figure;
% subplot(3,1,1); plot(tt, Qpp(1,:), 'LineWidth', 1.4); grid on;
% ylabel('q1̈ [rad/s^2]');
% subplot(3,1,2); plot(tt, Qpp(2,:), 'LineWidth', 1.4); grid on;
% ylabel('q2̈ [rad/s^2]');
% subplot(3,1,3); plot(tt, Qpp(3,:), 'LineWidth', 1.4); grid on;
% ylabel('q3̈ [rad/s^2]');
% xlabel('time [s]');
% sgtitle('Cubic spline trajectory – Joint Accelerations');
% 
% %% -------------------------------------------------------------
% %  PLOT TORQUES
% %% -------------------------------------------------------------
% figure;
% subplot(3,1,1); plot(tt, Fq_total(1,:), 'LineWidth', 1.4); grid on;
% ylabel('\tau_1 [Nm]');
% subplot(3,1,2); plot(tt, Fq_total(2,:), 'LineWidth', 1.4); grid on;
% ylabel('\tau_2 [Nm]');
% subplot(3,1,3); plot(tt, Fq_total(3,:), 'LineWidth', 1.4); grid on;
% ylabel('\tau_3 [Nm]');
% xlabel('time [s]');
% sgtitle('Joint Torques – Cubic spline trajectory');
% 
% %% -------------------------------------------------------------
% %  TCP PATH PLOT
% %% -------------------------------------------------------------
% figure; hold on; grid on; axis equal;
% plot(TCP(1,:), TCP(2,:), 'k-','LineWidth',2);
% title('TCP Trajectory – Cubic spline in joint space');
% xlabel('x [m]'); ylabel('y [m]');
% 
% %% -------------------------------------------------------------
% %  ANIMATION (same planar SCARA 3DOF)
% %% -------------------------------------------------------------
% figure; hold on; grid on; axis equal;
% title('SCARA 3DOF Animation – Cubic spline trajectory');
% 
% app = (l1+l2+l3)*1.2;
% xlim([-app app]); ylim([-app app]);
% 
% h = line([0 0],[0 0],'LineWidth',3,'Color','b',...
%          'Marker','o','MarkerFaceColor','k','MarkerSize',6);
% 
% for i = 1:10:n
%     q1 = Q(1,i); q2 = Q(2,i); q3 = Q(3,i);
% 
%     x1 = l1*cos(q1); y1 = l1*sin(q1);
%     x2 = x1 + l2*cos(q1+q2);
%     y2 = y1 + l2*sin(q1+q2);
%     x3 = x2 + l3*cos(q1+q2+q3);
%     y3 = y2 + l3*sin(q1+q2+q3);
% 
%     set(h,'XData',[0 x1 x2 x3], 'YData',[0 y1 y2 y3]);
%     drawnow;
% end
% 
% %% -------------------------------------------------------------
% %  EXPORT POSITIONS FOR SIMSCAPE (4 DOF, WITH OFFSETS)
% %% -------------------------------------------------------------
% disp("Exporting cubic-spline joint trajectories for Simscape (ADE)...");
% 
% % Offsets to match Simscape joint zero conventions
% offset_q1 = 0*deg2rad(90);
% offset_q2 = 0*deg2rad(180);
% offset_q3 = 0;
% offset_q4 = 0;   % base rotation stays as given
% 
% % Apply offsets
% q1_sim = Q(1,:) + offset_q1;
% q2_sim = Q(2,:) + offset_q2;
% q3_sim = Q(3,:) + offset_q3;
% q4_sim = Q(4,:);   % spline output already correct
% 
% % Helper function for Simscape format
% makeSignal = @(time, values) struct( ...
%     'time', time(:), ...
%     'signals', struct( ...
%         'values', values(:), ...
%         'dimensions', 1 ...
%     ) ...
% );
% 
% % Create Simscape-compatible structs
% q1_sig_ADE = makeSignal(tt, q1_sim);
% q2_sig_ADE = makeSignal(tt, q2_sim);
% q3_sig_ADE = makeSignal(tt, q3_sim);
% q4_sig_ADE = makeSignal(tt, q4_sim);
% 
% % Save to MAT file
% save('SCARA_spline_ADE.mat', ...
%      'q1_sig_ADE', 'q2_sig_ADE', 'q3_sig_ADE', 'q4_sig_ADE');
% 
% disp("Saved cubic spline ADE trajectory for Simscape: SCARA_spline_ADE.mat");
% 
% 


%% =============================================================
%   SCARA 3DOF – ADE trajectory using CUBIC SPLINES
%   4 DOF interpolation, 3 DOF dynamics, export for Simscape
% =============================================================
clc; clear; close all;

%% -------------------------------------------------------------
% ROBOT PARAMETERS
%% -------------------------------------------------------------
l1 = 0.24;
l2 = 0.2525;
l3 = 0.12;

g1 = 0.119; 
g2 = 0.063;
g3 = 0.03;

m1 = 1.62396;
m2 = 2.12630;
m3 = 0.00832;

I1 = 10911411.81e-9;
I2 = 14244066.35e-9;
I3 = 12179.49e-9;

L = [l1; l2; l3; g1; g2; g3; ...
     m1; m2; m3; I1; I2; I3];

%% -------------------------------------------------------------
%   ADE JOINT CONFIGURATIONS (Degrees → Radians)
%% -------------------------------------------------------------
Q2_deg = [ ...
     0         0        9.92       11.89;     % q1
    3.29     -25.13    -33.95     -55.73;     % q2
   -33.06    -29       -25.7      -15.21;     % q3
    -5.23     -5.23     -5.23      -5.23];    % q4

Qk_rad = deg2rad(Q2_deg);   % 4x4 matrix (all joints)

%% -------------------------------------------------------------
%   TIME GRID + SPLINE KNOTS
%% -------------------------------------------------------------
Ttot = 7.5;        
dt   = 0.01;
tt   = 0:dt:Ttot;
n    = length(tt);

tk = linspace(0, Ttot, 4);   % [0, 2, 4, 6]

%% -------------------------------------------------------------
%   SPLINE INTERPOLATION (ALL 4 JOINTS)
%% -------------------------------------------------------------
Q   = zeros(4,n);
Qp  = zeros(4,n);
Qpp = zeros(4,n);

for j = 1:4
    qj   = spline(tk, Qk_rad(j,:), tt);
    qjp  = gradient(qj, dt);
    qjpp = gradient(qjp, dt);

    Q(j,:)   = qj;
    Qp(j,:)  = qjp;
    Qpp(j,:) = qjpp;
end

% Keep only joints 1–3 for dynamic computation
Q_dyn   = Q(1:3,:);
Qp_dyn  = Qp(1:3,:);
Qpp_dyn = Qpp(1:3,:);

%% -------------------------------------------------------------
%   TCP POSITION (PLANAR 3 DOF)
%% -------------------------------------------------------------
TCP = zeros(2,n);
for i = 1:n
    q1 = Q_dyn(1,i); 
    q2 = Q_dyn(2,i); 
    q3 = Q_dyn(3,i);

    TCP(:,i) = [
        l1*cos(q1) + l2*cos(q1+q2) + l3*cos(q1+q2+q3);
        l1*sin(q1) + l2*sin(q1+q2) + l3*sin(q1+q2+q3)
    ];
end

%% -------------------------------------------------------------
%   DYNAMICS (3 DOF) + EXTERNAL FORCE
%% -------------------------------------------------------------
Fext = [0; -5; 0];   % 5N downward on TCP

Fq_total    = zeros(3,n);
Fq_inertia  = zeros(3,n);
Fq_coriolis = zeros(3,n);
Fq_gravity  = zeros(3,n);
Fq_ext      = zeros(3,n);

for i = 1:n
    q   = Q_dyn(:,i);
    qp  = Qp_dyn(:,i);
    qpp = Qpp_dyn(:,i);

    M  = SCARAM_3DOF(q, L);
    Cq = SCARAcoriolis_3DOF(q, qp, L);
    Gq = SCARAg_3DOF(q, L);
    J  = SCARAjac_3DOF(q, L);

    tau_ext = J' * Fext;

    Fq_inertia(:,i)  = M * qpp;
    Fq_coriolis(:,i) = Cq;
    Fq_gravity(:,i)  = Gq;
    Fq_ext(:,i)      = tau_ext;

    Fq_total(:,i) = Fq_inertia(:,i) + Cq + Gq + tau_ext;
end

%% -------------------------------------------------------------
%   PLOTS – q(t), qdot(t), qddot(t)
%% -------------------------------------------------------------
figure;
subplot(4,1,1); plot(tt, Q(1,:), 'LineWidth',1.4); grid on; ylabel('q1 [rad]');
subplot(4,1,2); plot(tt, Q(2,:), 'LineWidth',1.4); grid on; ylabel('q2 [rad]');
subplot(4,1,3); plot(tt, Q(3,:), 'LineWidth',1.4); grid on; ylabel('q3 [rad]');
subplot(4,1,4); plot(tt, Q(4,:), 'LineWidth',1.4); grid on; ylabel('q4 [rad]');
xlabel('time [s]'); sgtitle('Joint Positions – ADE');

figure;
subplot(3,1,1); plot(tt, Qp_dyn(1,:), 'LineWidth',1.4); grid on; ylabel('q1̇');
subplot(3,1,2); plot(tt, Qp_dyn(2,:), 'LineWidth',1.4); grid on; ylabel('q2̇');
subplot(3,1,3); plot(tt, Qp_dyn(3,:), 'LineWidth',1.4); grid on; ylabel('q3̇');
xlabel('time [s]'); sgtitle('Joint Velocities – ADE');

figure;
subplot(3,1,1); plot(tt, Qpp_dyn(1,:), 'LineWidth',1.4); grid on; ylabel('q1̈');
subplot(3,1,2); plot(tt, Qpp_dyn(2,:), 'LineWidth',1.4); grid on; ylabel('q2̈');
subplot(3,1,3); plot(tt, Qpp_dyn(3,:), 'LineWidth',1.4); grid on; ylabel('q3̈');
xlabel('time [s]'); sgtitle('Joint Accelerations – ADE');

%% -------------------------------------------------------------
%   TORQUES
%% -------------------------------------------------------------
figure;
subplot(3,1,1); plot(tt, Fq_total(1,:), 'LineWidth',1.4); grid on; ylabel('\tau_1');
subplot(3,1,2); plot(tt, Fq_total(2,:), 'LineWidth',1.4); grid on; ylabel('\tau_2');
subplot(3,1,3); plot(tt, Fq_total(3,:), 'LineWidth',1.4); grid on; ylabel('\tau_3');
xlabel('time [s]'); sgtitle('Joint Torques – ADE');

%% -------------------------------------------------------------
%   TCP PATH
%% -------------------------------------------------------------
figure; hold on; grid on; axis equal;
plot(TCP(1,:), TCP(2,:), 'k-', 'LineWidth',2);
title('TCP Path – ADE Spline');
xlabel('x [m]'); ylabel('y [m]');

%% -------------------------------------------------------------
%   ANIMATION
%% -------------------------------------------------------------
figure; hold on; grid on; axis equal;
title('SCARA Animation – ADE trajectory');

app = (l1 + l2 + l3) * 1.2;
xlim([-app app]); ylim([-app app]);

h = line([0 0],[0 0], 'LineWidth',3, 'Color','b', ...
         'Marker','o','MarkerFaceColor','k');

for i = 1:10:n
    q1=Q_dyn(1,i); q2=Q_dyn(2,i); q3=Q_dyn(3,i);

    x1 = l1*cos(q1);        y1 = l1*sin(q1);
    x2 = x1 + l2*cos(q1+q2); y2 = y1 + l2*sin(q1+q2);
    x3 = x2 + l3*cos(q1+q2+q3); y3 = y2 + l3*sin(q1+q2+q3);

    set(h,'XData',[0 x1 x2 x3], 'YData',[0 y1 y2 y3]);
    drawnow;
end

%% -------------------------------------------------------------
%   EXPORT FOR SIMSCAPE – ALL 4 JOINTS
%% -------------------------------------------------------------
disp("Exporting ADE spline trajectory for Simscape...");

offset_q1 = 0;
offset_q2 = 0*deg2rad(180);
offset_q3 = 0*deg2rad(180);
offset_q4 = 0;

q1_sim = Q(1,:) + offset_q1;
q2_sim = Q(2,:) + offset_q2;
q3_sim = Q(3,:) + offset_q3;
q4_sim = Q(4,:) + offset_q4;

makeSignal = @(t,val) struct( ...
    'time', t(:), ...
    'signals', struct('values', val(:), 'dimensions',1));

q1_sig_ADE = makeSignal(tt, q1_sim);
q2_sig_ADE = makeSignal(tt, q2_sim);
q3_sig_ADE = makeSignal(tt, q3_sim);
q4_sig_ADE = makeSignal(tt, q4_sim);

save("SCARA_spline_ADE.mat", ...
    "q1_sig_ADE","q2_sig_ADE","q3_sig_ADE","q4_sig_ADE");

disp("Saved: SCARA_spline_ADE.mat");
