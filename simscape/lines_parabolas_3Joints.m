function traj = lines_parabolas_3Joints()

close all;
clc;

%% -------------------------------------------------------------
%  1) DATI DEL ROBOT (GIUNTI MISURATI DAL CAD)
%     Angoli in GRADI → convertiti in radianti
% --------------------------------------------------------------

Q_deg = [ ...
    -24      -68.88    -42.64    -45.39;     % q2 
    -35.52    29.84     6.44      36.03;   % q3 
     21.34    39.04     36.2     9.37 ];  % q4

Q = deg2rad(Q_deg);   % conversione in rad
[n_joints, n_points] = size(Q);

%% --------------------------------------------------------------
% 2) PARAMETRI DEL GIUNTO (limitazioni fisiche)
% --------------------------------------------------------------

motor.A = 7;    % accel max rad/s^2
motor.D = 7;    % decel max rad/s^2
motor.V = 10;     % velocità max rad/s

%% --------------------------------------------------------------
% 3) CALCOLO DEGLI SPOSTAMENTI PER TUTTI I TRATTI
% --------------------------------------------------------------

dQ = diff(Q, 1, 2);   % differenze tra colonne
n_segments = n_points - 1;  % es: 4 punti = 3 segmenti

%% --------------------------------------------------------------
% 4) TEMPO MINIMO DI ATTUAZIONE PER OGNI SEGMENTO
% --------------------------------------------------------------

T_rise = zeros(n_joints, n_segments);

for j = 1:n_joints
    for k = 1:n_segments
        T_rise(j,k) = min_rise_time_custom(motor, abs(dQ(j,k)));
    end
end

% tempo per ogni segmento scelto come max sui giunti
T_min_act = max(T_rise, [], 1);

%% --------------------------------------------------------------
% 5) GENERAZIONE DELLA TRAIETTORIA COMPLETA
% --------------------------------------------------------------

T_total = sum(T_min_act);
dt = 0.001;
t = 0:dt:T_total;
N = length(t);

Q_traj = zeros(n_joints, N);

t_current = 0;
index_start = 1;

for seg = 1:n_segments

    Tseg = T_min_act(seg);
    t_end = t_current + Tseg;

    for j = 1:n_joints

        [Tacc, Tdec, Vpeak] = trapezoidal_custom(motor, dQ(j,seg), Tseg);

        for i = index_start:N

            if t(i) > t_end
                break
            end

            tau = t(i) - t_current;

            if tau <= Tacc
                % Parabola accelerazione
                q = Q(j,seg) + sign(dQ(j,seg)) * (motor.A * tau^2 / 2);

            elseif tau <= Tseg - Tdec
                % Tratto lineare
                q = Q(j,seg) + sign(dQ(j,seg)) * ...
                    (motor.A*Tacc^2/2 + Vpeak*(tau - Tacc));

            else
                % Parabola decelerazione
                td = tau - (Tseg - Tdec);
                q = Q(j,seg+1) - sign(dQ(j,seg)) * (motor.D * (Tdec - td)^2 / 2);
            end

            Q_traj(j,i) = q;
        end
    end

    % Aggiorna indici per segmento successivo
    while index_start <= N && t(index_start) < t_end
        index_start = index_start + 1;
    end

    t_current = t_end;
end

%% --------------------------------------------------------------
%  OUTPUT → variabile traj da usare in SIMSCAPE
% --------------------------------------------------------------

traj = [t' Q_traj'];
assignin('base', 'traj', traj);

disp('Traiettoria aggiornata generata! Variabile "traj" disponibile nel workspace.');

end