

% ===== Time =====
t = out.Torques_value.time;    % Nx1

% ===== ACCELERAZIONI GIUNTI (rad/s^2) =====
acc_joint1 = out.acc_values.signals.values(:,1);
acc_joint2 = out.acc_values.signals.values(:,2);
acc_joint3 = out.acc_values.signals.values(:,3);
acc_joint4 = out.acc_values.signals.values(:,4);

% ===== COPPIE GIUNTI da Simscape (Nm) =====
torque_joint1 = out.Torques_value.signals.values(:,1);
torque_joint2 = out.Torques_value.signals.values(:,2);
torque_joint3 = out.Torques_value.signals.values(:,3);
torque_joint4 = out.Torques_value.signals.values(:,4);

% ===== Durata performance =====
t_a = t(end) - t(1);

% ===== RMS accelerazioni =====
acc_rms_joint2 = sqrt( (1/t_a) * trapz(t, acc_joint2.^2) );
acc_rms_joint3 = sqrt( (1/t_a) * trapz(t, acc_joint3.^2) );
acc_rms_joint4 = sqrt( (1/t_a) * trapz(t, acc_joint4.^2) );

% ===== RMS coppie (C_r^*) =====
torque_rms_joint2 = sqrt( (1/t_a) * trapz(t, torque_joint2.^2) );
torque_rms_joint3 = sqrt( (1/t_a) * trapz(t, torque_joint3.^2) );
torque_rms_joint4 = sqrt( (1/t_a) * trapz(t, torque_joint4.^2) );


% Media del prodotto (acc * torque) sul tempo di performance
meanProd_joint2 = (1/t_a) * trapz(t, acc_joint2 .* torque_joint2);
meanProd_joint3 = (1/t_a) * trapz(t, acc_joint3 .* torque_joint3);
meanProd_joint4 = (1/t_a) * trapz(t, acc_joint4 .* torque_joint4);

% ===== Beta (scalari, uno per giunto) =====
beta_joint2 = 2 * (acc_rms_joint2 * torque_rms_joint2 + meanProd_joint2);
beta_joint3 = 2 * (acc_rms_joint3 * torque_rms_joint3 + meanProd_joint3);
beta_joint4 = 2 * (acc_rms_joint4 * torque_rms_joint4 + meanProd_joint4);

beta = [beta_joint2 beta_joint3 beta_joint4]


% ===== DATI MOTORI =====
Cn_joint2 = 0.32;     % [Nm]
Cn_joint4 = 0.32;     % [Nm]
Cn_joint3 = 0.64;     % [Nm]

Jm_joint2 = 0.03e-4;  % [kg*m^2]
Jm_joint4 = 0.03e-4;  % [kg*m^2]
Jm_joint3 = 0.0865e-4;% [kg*m^2]

% ===== ALPHA =====
alpha_joint2 = (Cn_joint2^2) / Jm_joint2;
alpha_joint3 = (Cn_joint3^2) / Jm_joint3;
alpha_joint4 = (Cn_joint4^2) / Jm_joint4;

alpha = [alpha_joint2 alpha_joint3 alpha_joint4]

% ===== TAU OPT (rapporto di trasmissione ottimo) =====
tauOpt_joint2 = sqrt( Jm_joint2 * (acc_rms_joint2 / torque_rms_joint2) );
tauOpt_joint3 = sqrt( Jm_joint3 * (acc_rms_joint3 / torque_rms_joint3) );
tauOpt_joint4 = sqrt( Jm_joint4 * (acc_rms_joint4 / torque_rms_joint4) );

tau_opt = [tauOpt_joint2 tauOpt_joint3 tauOpt_joint4];

N_opt = 1 ./ tau_opt;   % riduzione "classica" equivalente

% Rapporti REALI (meccanici)
N_real   = [16 4 4];
tau_real = 1 ./ N_real;


% Dati (metti i tuoi valori)
beta  = [14.2701 4.5819 0.0083];                 % [joint2 joint3 joint4]
alpha = 1e4 * [3.4133 4.7353 3.4133];            % come da output MATLAB


% ===== TAU MIN e TAU MAX (formula slide) =====

% vettori comodi
acc_rms = [acc_rms_joint2 acc_rms_joint3 acc_rms_joint4];
torque_rms = [torque_rms_joint2 torque_rms_joint3 torque_rms_joint4];
Jm = [Jm_joint2 Jm_joint3 Jm_joint4];

% termini della formula
D = alpha - beta;   % (alpha - beta)

tau_min = sqrt(Jm) .* sqrt( D + 4 .* acc_rms .* torque_rms - sqrt(D) ) ...
          ./ (2 .* torque_rms);

tau_max = sqrt(Jm) .* sqrt( D + 4 .* acc_rms .* torque_rms + sqrt(D) ) ...
          ./ (2 .* torque_rms);



joint = [2; 3; 4]';
margin = alpha ./ beta;                          % alpha/beta

T = table(joint(:), ...
          alpha(:), ...
          beta(:), ...
          margin(:), ...
          tau_min(:), ...
          tau_opt(:), ...
          tau_max(:), ...
          N_opt(:), ...
          tau_real(:), ...
          N_real(:), ...
          'VariableNames', ...
          {'Joint', ...
           'Alpha [W/s]', ...
           'Beta [W/s]', ...
           'Alpha_over_Beta', ...
           'Tau_min', ...
           'Tau_opt', ...
           'Tau_max', ...
           'N_opt', ...
           'Tau_real', ...
           'N_real'});

disp(T)