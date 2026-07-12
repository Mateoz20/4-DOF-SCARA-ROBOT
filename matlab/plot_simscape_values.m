%% ================================================================
%   EXTRACT FROM SIMSCAPE: ANGLES, VELOCITIES, ACCELERATIONS, TORQUES
% ================================================================

%% -------- ANGLES -------------------------------------------------
t_sim = out.angle_values.time;       % Time vector (same for all)
A = out.angle_values.signals.values; % Nx4 matrix

q1_sim = A(:,1);
q2_sim = A(:,2);
q3_sim = A(:,3);
q4_sim = A(:,4);

%% -------- VELOCITIES --------------------------------------------
V = out.speed_values.signals.values; % Nx4 matrix

q1dot_sim = V(:,1);
q2dot_sim = V(:,2);
q3dot_sim = V(:,3);
q4dot_sim = V(:,4);

%% -------- ACCELERATIONS -----------------------------------------
Acc = out.acc_values.signals.values; % Nx4 matrix

q1dd_sim = Acc(:,1);
q2dd_sim = Acc(:,2);
q3dd_sim = Acc(:,3);
q4dd_sim = Acc(:,4);

%% -------- TORQUES -----------------------------------------------
T = out.Torques_value.signals.values; % Nx4 matrix

tau1_sim = T(:,1);
tau2_sim = T(:,2);
tau3_sim = T(:,3);
tau4_sim = T(:,4);

%% ================================================================
%   SAVE RESULTS TO .MAT FILE
% ================================================================
save('simscape_joint_data.mat', ...
    't_sim', ...
    'q1_sim','q2_sim','q3_sim','q4_sim', ...
    'q1dot_sim','q2dot_sim','q3dot_sim','q4dot_sim', ...
    'q1dd_sim','q2dd_sim','q3dd_sim','q4dd_sim', ...
    'tau1_sim','tau2_sim','tau3_sim','tau4_sim');

disp('Saved: simscape_joint_data.mat');


%% ================================================================
%   OPTIONAL QUICK PLOTS
% ================================================================

figure;
subplot(4,1,1); plot(t_sim, q1_sim); grid on; ylabel('q1 [rad]');
subplot(4,1,2); plot(t_sim, q2_sim); grid on; ylabel('q2 [rad]');
subplot(4,1,3); plot(t_sim, q3_sim); grid on; ylabel('q3 [rad]');
subplot(4,1,4); plot(t_sim, q4_sim); grid on; ylabel('q4 [rad]');
xlabel('time [s]');
sgtitle('Joint Angles from Simscape');

figure;
subplot(4,1,1); plot(t_sim, tau1_sim); grid on; ylabel('\tau_1 [Nm]');
subplot(4,1,2); plot(t_sim, tau2_sim); grid on; ylabel('\tau_2 [Nm]');
subplot(4,1,3); plot(t_sim, tau3_sim); grid on; ylabel('\tau_3 [Nm]');
subplot(4,1,4); plot(t_sim, tau4_sim); grid on; ylabel('\tau_4 [Nm]');
xlabel('time [s]');
sgtitle('Joint Torques from Simscape');


