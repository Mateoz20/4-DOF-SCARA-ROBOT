function traj2 = traj_cubic_4Joints()
% Traiettoria 2: polinomi cubici (4 giunti)
% OUTPUT: traj2 = [t q1 q2 q3 q4] in radianti

close all; clc;

%% -------------------------------------------------------------
% 1) Via-points misurati dal CAD (in GRADI)
%     punti:  P1     P2      P3      P4
% --------------------------------------------------------------

Q2_deg = [ ...
     0        0      13.15    15.17;     % q1
   -12.56  -36.41   -43.51   -74.73;     % q2
   -41.24  -17.85     6.85    60.09;     % q3
     0        0        0        0];    % q4

% Conversione in radianti
Q = deg2rad(Q2_deg);

[n_joints, n_points] = size(Q);
n_segments = n_points - 1;

%% -------------------------------------------------------------
% 2) Assegnazione dei tempi per ogni via-point (modificabile)
% --------------------------------------------------------------

T_total = 2.0;                     % durata totale (s)
Tseg = T_total / n_segments;       % durata di ogni segmento
t_k = (0:n_segments) * Tseg;       % tempi dei via-points


%% -------------------------------------------------------------
% 3) Calcolo delle velocità nei via-points
%    v(1)=0, v(end)=0, interne = derivata centrale
% --------------------------------------------------------------

dq_k = zeros(n_joints, n_points);

for j = 1:n_joints
    dq_k(j,1)   = 0;   % velocità iniziale
    dq_k(j,end) = 0;   % velocità finale

    for k = 2:n_points-1
        dq_k(j,k) = (Q(j,k+1) - Q(j,k-1)) / (t_k(k+1) - t_k(k-1));
    end
end


%% -------------------------------------------------------------
% 4) Costruzione traiettoria continua
% --------------------------------------------------------------

dt = 0.001;
t = 0:dt:T_total;
N = length(t);

Q_traj = zeros(n_joints, N);

for seg = 1:n_segments

    t0 = t_k(seg);
    tf = t_k(seg+1);
    Tseg = tf - t0;

    idx = find(t >= t0 & t <= tf); 
    tau = t(idx) - t0;

    for j = 1:n_joints

        q0  = Q(j,seg);
        qf  = Q(j,seg+1);
        dq0 = dq_k(j,seg);
        dqf = dq_k(j,seg+1);

        % Coefficienti polinomio cubico
        a0 = q0;
        a1 = dq0;
        a2 = (3*(qf - q0)/Tseg^2) - (2*dq0 + dqf)/Tseg;
        a3 = (2*(q0 - qf)/Tseg^3) + (dq0 + dqf)/Tseg^2;

        Q_traj(j,idx) = a0 + a1*tau + a2*tau.^2 + a3*tau.^3;
    end
end


%% -------------------------------------------------------------
% 5) OUTPUT per Simulink
% --------------------------------------------------------------

traj2 = [t' Q_traj'];
assignin('base','traj2',traj2);


%% -------------------------------------------------------------
% 6) Plot di verifica
% --------------------------------------------------------------

figure;
plot(traj2(:,1), traj2(:,2:5),'LineWidth',1.5);
xlabel('t [s]'); ylabel('q [rad]');
grid on; legend('q1','q2','q3','q4');
title('Traiettoria 2 - Polinomi Cubici');

end
