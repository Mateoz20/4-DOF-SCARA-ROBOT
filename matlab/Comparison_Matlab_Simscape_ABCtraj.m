% Comparison ABC Trajectory-Simscape/Matlab

figure();
subplot(6,2,1); plot(tt,Q(1,:),'LineWidth',1.4); grid on; ylabel('q2 [rad]');
subplot(6,2,3); plot(tt,Q(2,:),'LineWidth',1.4); grid on; ylabel('q3 [rad]');
subplot(6,2,5); plot(tt,Q(3,:),'LineWidth',1.4); grid on; ylabel('q4 [rad]');
%subplot(4,1,1); plot(t_sim, q1_sim); grid on; ylabel('q1 [rad]');
subplot(6,2,2); plot(t_sim, q2_sim - 4.71239); grid on; ylabel('q2 [rad]');
subplot(6,2,4); plot(t_sim, q3_sim - 1.571); grid on; ylabel('q3 [rad]');
subplot(6,2,6); plot(t_sim, q4_sim); grid on; ylabel('q4 [rad]');
xlabel('time [s]');sgtitle('Joint Positions (Matlab vs Simscape) ABC Trajectory');

figure();
subplot(6,2,1); plot(tt,Qp(1,:),'LineWidth',1.4); grid on; ylabel('q2̇p [rad/s]');
subplot(6,2,3); plot(tt,Qp(2,:),'LineWidth',1.4); grid on; ylabel('q3ṗ [rad/s]');
subplot(6,2,5); plot(tt,Qp(3,:),'LineWidth',1.4); grid on; ylabel('q4ṗ [rad/s]');
xlabel('time [s]');
subplot(6,2,2); plot(t_sim, q3dot_sim); grid on; ylabel('q2p [rad/s]');
subplot(6,2,4); plot(t_sim, q4dot_sim); grid on; ylabel('q3p [rad/s]');
subplot(6,2,6); plot(t_sim, q1dot_sim); grid on; ylabel('q4p [rad/s]');
xlabel('time [s]'); sgtitle('Joint Velocities (Matlab vs Simscape) ABC Trajectory');


figure();
subplot(6,2,1); plot(tt,Qpp(1,:),'LineWidth',1.4); grid on; ylabel('q2̇pp [rad/s^2]');
subplot(6,2,3); plot(tt,Qpp(2,:),'LineWidth',1.4); grid on; ylabel('q3pṗ [rad/s^2]');
subplot(6,2,5); plot(tt,Qpp(3,:),'LineWidth',1.4); grid on; ylabel('q4ṗp [rad/s^2]');
xlabel('time [s]');
subplot(6,2,2); plot(t_sim, q3dd_sim); grid on; ylabel('q2pp [rad/s^2]');
subplot(6,2,4); plot(t_sim, q4dd_sim); grid on; ylabel('q3pp [rad/s^2]');
subplot(6,2,6); plot(t_sim, q1dd_sim); grid on; ylabel('q4pp [rad/s^2]');
xlabel('time [s]'); sgtitle('Joint Acceleration (Matlab vs Simscape) ABC Trajectory');


figure();
subplot(6,2,1); plot(tt,-Fq(1,:),'LineWidth',1.4); grid on; ylabel('\tau_1 [Nm]');
subplot(6,2,3); plot(tt,-Fq(2,:),'LineWidth',1.4); grid on; ylabel('\tau_2 [Nm]');
subplot(6,2,5); plot(tt,-Fq(3,:),'LineWidth',1.4); grid on; ylabel('\tau_3 [Nm]');
xlabel('time [s]'); 
subplot(6,2,2); plot(t_sim, tau2_sim); grid on; ylabel('\tau_2 [Nm]');
subplot(6,2,4); plot(t_sim, tau3_sim); grid on; ylabel('\tau_3 [Nm]');
subplot(6,2,6); plot(t_sim, tau4_sim); grid on; ylabel('\tau_4 [Nm]');
xlabel('time [s]');sgtitle('Joint Torques (Matlab vs Simscape) ABC Trajectory - 0.5 Kg');



%% ======================================================
%     TORQUE MATLAB vs SIMSCAPE + PERCENTAGE ERROR
% ======================================================

% Extract MATLAB torques (NOTE: you used -Fq in plot → keep that convention)
tau1_mat = -Fq(1,:)';
tau2_mat = -Fq(2,:)';
tau3_mat = -Fq(3,:)';

% Resize all series to sync lengths
N = min([length(tau1_mat), length(tau1_sim), length(t_sim), length(tt)]);
tau1_mat = tau1_mat(1:N);  tau2_mat = tau2_mat(1:N);  tau3_mat = tau3_mat(1:N);
tau1_sim = tau1_sim(1:N);  tau2_sim = tau2_sim(1:N);  tau3_sim = tau3_sim(1:N);
t_sim = t_sim(1:N);        tt = tt(1:N);

% Prevent division by zero
eps_reg = 1e-9;

% Percentage error
err1 = abs(tau1_mat - tau1_sim) ./ max(abs(tau1_sim), eps_reg) * 100;
err2 = abs(tau2_mat - tau2_sim) ./ max(abs(tau2_sim), eps_reg) * 100;
err3 = abs(tau3_mat - tau3_sim) ./ max(abs(tau3_sim), eps_reg) * 100;


% === Percentage errors ===
subplot(6,2,7); plot(t_sim, err1, 'LineWidth',1.4); grid on; ylabel('Err_1 [%]');
subplot(6,2,9); plot(t_sim, err2, 'LineWidth',1.4); grid on; ylabel('Err_2 [%]');
subplot(6,2,11); plot(t_sim, err3, 'LineWidth',1.4); grid on; ylabel('Err_3 [%]'); xlabel('time [s]');