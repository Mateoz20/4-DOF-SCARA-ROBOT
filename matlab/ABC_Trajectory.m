%% =============================================================
%   SCARA 3DOF – TRAJECTORY USING LINES & PARABOLAS (TRAPEZOIDAL)
%   Four Cartesian points -> IK -> Joint-space trapezoidal profile
%   Complete dynamics + 5N external force + plots + animation
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

L = [l1; l2; l3; g1; g2; g3; m1; m2; m3; I1; I2; I3];

%% -------------------------------------------------------------
%  FOUR CARTESIAN POINTS (mm -> m)
%% -------------------------------------------------------------
P1 = [300, -70]/1000;
P2 = [500, -70]/1000;
P3 = [440,  15]/1000;
P4 = [500, 100]/1000;

Phi = 0;

S_list = [P1' P2' P3' P4'];

%% -------------------------------------------------------------
% INVERSE KINEMATICS FOR THE 4 POINTS
%% -------------------------------------------------------------
Qk = zeros(3,4);
for i = 1:4
    Qk(:,i) = SCARAinv_3DOF(S_list(:,i), Phi, L, -1);
end

Qk = unwrap(Qk,[],2);   % avoid 2π jumps

%% -------------------------------------------------------------
% JOINT DELTAS (3 segments)
%% -------------------------------------------------------------
dQ = diff(Qk,1,2);  % 3x3

%% -------------------------------------------------------------
% MOTOR PARAMETERS
%% -------------------------------------------------------------
% motor.A = 1;      % acceleration  [rad/s^2]
% motor.D = 1;      % deceleration  [rad/s^2]
% motor.V = 0.5;      % not used directly



motor.A = 2.5;      % acceleration  [rad/s^2]
motor.D = 2.5;      % deceleration  [rad/s^2]
motor.V = 100;      % unused but must be large so we stay in case 1

%% -------------------------------------------------------------
% COMPUTE SEGMENT TIME USING PROFESSOR'S RULE
%% -------------------------------------------------------------
rise_time = zeros(3,3);

for j = 1:3
    for s = 1:3
        rise_time(j,s) = min_rise_time(motor, abs(dQ(j,s)));
    end
end

Tseg = max(rise_time(:));    % same time for all joints and segments
dt   = 0.01;

%% -------------------------------------------------------------
% TRAJECTORY CONSTRUCTION USING TRAPEZOIDAL PROFILE
%% -------------------------------------------------------------
tt = [];
Q = []; Qp = []; Qpp = [];

for s = 1:3

    q0 = Qk(:,s);
    qf = Qk(:,s+1);

    % Joint 1
    [t1, q1, q1p, q1pp] = trapezoidal_profile(q0(1), qf(1), Tseg, motor.A, motor.D, dt);
    % Joint 2
    [~,  q2, q2p, q2pp] = trapezoidal_profile(q0(2), qf(2), Tseg, motor.A, motor.D, dt);
    % Joint 3
    [~,  q3, q3p, q3pp] = trapezoidal_profile(q0(3), qf(3), Tseg, motor.A, motor.D, dt);

    if s > 1
        t1 = t1 + tt(end) + dt;   % shift time
    end

    tt  = [tt,  t1];
    Q   = [Q,   [q1;  q2;  q3]];
    Qp  = [Qp,  [q1p; q2p; q3p]];
    Qpp = [Qpp, [q1pp;q2pp;q3pp]];
end

n = length(tt);

%% -------------------------------------------------------------
%  TCP TRAJECTORY (for plotting)
%% -------------------------------------------------------------
TCP = zeros(2,n);
for i = 1:n
    q1 = Q(1,i); q2 = Q(2,i); q3 = Q(3,i);
    TCP(:,i) = [
        l1*cos(q1) + l2*cos(q1+q2) + l3*cos(q1+q2+q3);
        l1*sin(q1) + l2*sin(q1+q2) + l3*sin(q1+q2+q3)
    ];
end

%% -------------------------------------------------------------
% DYNAMICS + EXTERNAL FORCE (5 N DOWNWARD)
%% -------------------------------------------------------------
Fext = [0; -0.5*9.8; 0];
Fq = zeros(3,n);

for i = 1:n
    q = Q(:,i);
    qp = Qp(:,i);
    qpp = Qpp(:,i);

    M  = SCARAM_3DOF(q,L);
    Cq = SCARAcoriolis_3DOF(q,qp,L);
    Gq = SCARAg_3DOF(q,L);
    J  = SCARAjac_3DOF(q,L);

    tau_ext = J' * Fext;

    Fq(:,i) = M*qpp + Cq + Gq + tau_ext;
end

%% -------------------------------------------------------------
%            PLOTS – JOINT POSITION, VELOCITY, ACCELERATION
%% -------------------------------------------------------------
figure;
subplot(3,1,1); plot(tt,Q(1,:),'LineWidth',1.4); grid on; ylabel('q1 [rad]');
subplot(3,1,2); plot(tt,Q(2,:),'LineWidth',1.4); grid on; ylabel('q2 [rad]');
subplot(3,1,3); plot(tt,Q(3,:),'LineWidth',1.4); grid on; ylabel('q3 [rad]');
xlabel('time [s]'); sgtitle('Joint Positions');

figure;
subplot(3,1,1); plot(tt,Qp(1,:),'LineWidth',1.4); grid on; ylabel('q1̇p [rad/s]');
subplot(3,1,2); plot(tt,Qp(2,:),'LineWidth',1.4); grid on; ylabel('q2ṗ [rad/s]');
subplot(3,1,3); plot(tt,Qp(3,:),'LineWidth',1.4); grid on; ylabel('q3ṗ [rad/s]');
xlabel('time [s]'); sgtitle('Joint Velocities');

figure;
subplot(3,1,1); plot(tt,Qpp(1,:),'LineWidth',1.4); grid on; ylabel('q1pp [rad/s^2]');
subplot(3,1,2); plot(tt,Qpp(2,:),'LineWidth',1.4); grid on; ylabel('q2̈pp [rad/s^2]');
subplot(3,1,3); plot(tt,Qpp(3,:),'LineWidth',1.4); grid on; ylabel('q3̈pp [rad/s^2]');
xlabel('time [s]'); sgtitle('Joint Accelerations');

%% -------------------------------------------------------------
%  PLOT TORQUES
%% -------------------------------------------------------------
figure;
subplot(3,1,1); plot(tt,-Fq(1,:),'LineWidth',1.4); grid on; ylabel('\tau_1 [Nm]');
subplot(3,1,2); plot(tt,-Fq(2,:),'LineWidth',1.4); grid on; ylabel('\tau_2 [Nm]');
subplot(3,1,3); plot(tt,-Fq(3,:),'LineWidth',1.4); grid on; ylabel('\tau_3 [Nm]');
xlabel('time [s]'); sgtitle('Joint Torques (incl. external force)');

%% -------------------------------------------------------------
%  TCP PATH PLOT
%% -------------------------------------------------------------
figure; hold on; grid on; axis equal;
plot(TCP(1,:), TCP(2,:), 'k--','LineWidth',2);
scatter(S_list(1,:), S_list(2,:), 70,'r','filled');
title('TCP Trajectory – Lines & Parabolas');
xlabel('x [m]'); ylabel('y [m]');

%% -------------------------------------------------------------
%  ANIMATION
%% -------------------------------------------------------------
figure; hold on; grid on; axis equal;
title('SCARA 3DOF Animation – Lines & Parabolas');
plot(TCP(1,:), TCP(2,:), 'k--','LineWidth',2);

app = (l1+l2+l3)*1.2;
xlim([-app app]); ylim([-app app]);

h = line([0 0],[0 0],'LineWidth',3,'Color','b',...
         'Marker','o','MarkerFaceColor','k','MarkerSize',6);

for i = 1:10:n
    q1 = Q(1,i); q2 = Q(2,i); q3 = Q(3,i);

    x1 = l1*cos(q1); y1 = l1*sin(q1);
    x2 = x1 + l2*cos(q1+q2);
    y2 = y1 + l2*sin(q1+q2);
    x3 = x2 + l3*cos(q1+q2+q3);
    y3 = y2 + l3*sin(q1+q2+q3);

    set(h,'XData',[0 x1 x2 x3], 'YData',[0 y1 y2 y3]);
    drawnow;
end


%% ============================================================
%   SAVE TRAJECTORY FOR SIMSCAPE (with joint angle offsets)
% ============================================================

% Offsets needed because Simscape uses different joint reference frames
offset_q1 = deg2rad(270);     % +90° for joint 1
offset_q2 = deg2rad(90);    % +180° for joint 2
offset_q3 = 0;               % no change for joint 3

% Apply offsets for Simscape motion inputs
q1_sim = Q(1,:) + offset_q1;
q2_sim = Q(2,:) + offset_q2;
q3_sim = Q(3,:) + offset_q3;

%% Create signals for Simscape "From Workspace" blocks
q1_sig.time                = tt';
q1_sig.signals.values      = q1_sim';
q1_sig.signals.dimensions  = 1;

q2_sig.time                = tt';
q2_sig.signals.values      = q2_sim';
q2_sig.signals.dimensions  = 1;

q3_sig.time                = tt';
q3_sig.signals.values      = q3_sim';
q3_sig.signals.dimensions  = 1;

% Save everything for Simscape
save('SCARA_pos_simscape.mat','q1_sig','q2_sig','q3_sig');

disp("Trajectory for Simscape saved as: SCARA_pos_simscape.mat");

