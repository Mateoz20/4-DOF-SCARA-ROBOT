function [Tacc, Tdec, Vpeak] = trapezoidal_custom(motor, dq, T)
% dq = spostamento giunto nel tratto
% T = durata totale del tratto (T_min_act)

dq = abs(dq);
A = motor.A;
D = motor.D;

% calcolo Vpeak imponendo che il trapezio duri esattamente T
Vpeak = (0.5*A*T) - sqrt((0.5*A*T)^2 - A*dq);

Tacc = Vpeak/A;
Tdec = Vpeak/D;
end
