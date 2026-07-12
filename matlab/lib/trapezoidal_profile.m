function [t, q, qd, qdd] = trapezoidal_profile(q0, qf, T, A, D, dt)
% ------------------------------------------------------------
% TRAPEZOIDAL MOTION PROFILE (lines + parabolas)
% 
% Motion:
%   1) Accelerate with constant +A      for time ta
%   2) Move at constant velocity vmax   for time T_const
%   3) Decelerate with constant -D      for time td
%
% The times ta and td are computed so that:
%   - total time  = T
%   - total travel = |qf - q0|
%   - velocity is continuous
%
% INPUT:
%   q0  : initial position
%   qf  : final position
%   T   : total motion time
%   A   : acceleration  ( > 0 )
%   D   : deceleration  ( > 0 )
%   dt  : time step
%
% OUTPUT:
%   t   : time vector
%   q   : position
%   qd  : velocity
%   qdd : acceleration
% ------------------------------------------------------------

% time vector
t = 0:dt:T;
N = length(t);

% total displacement and sign
dq  = qf - q0;
sgn = sign(dq);
Dq  = abs(dq);

% ------------------------------------------------------------
% Compute ta (acceleration time) from exact equation:
%   Dq = 0.5*A*ta^2 + (A*ta)*T_const + 0.5*D*td^2
% with
%   td     = (A/D)*ta  (same vmax in accel and decel)
%   T_const = T - ta - td
%
% After some algebra this gives:
%   Dq = A*ta*(D*T - 0.5*(A + D)*ta)/D
%
% We solve this for ta and choose the smaller positive root.
% ------------------------------------------------------------

% discriminant of the quadratic (must be >= 0 for a feasible motion)
disc = A*D*(A*D*T^2 - 2*(A + D)*Dq);

if disc < 0
    error('trapezoidal_profile: infeasible motion (T too small or A,D too low)');
end

% smaller positive root for ta
ta = (A*D*T - sqrt(disc)) / (A*(A + D));

% corresponding deceleration time
td = (A/D)*ta;

% constant-velocity phase duration
T_const = T - ta - td;

% maximum velocity
vmax = A * ta;

% ------------------------------------------------------------
% Preallocate arrays
% ------------------------------------------------------------
q   = zeros(1,N);
qd  = zeros(1,N);
qdd = zeros(1,N);

% ------------------------------------------------------------
% Build the profile
% ------------------------------------------------------------
for k = 1:N
    tk = t(k);

    % ---------- ACCELERATION PHASE ----------
    if tk <= ta
        q_rel  = 0.5 * A * tk^2;
        v_rel  = A * tk;
        a_rel  = A;

    % ---------- CONSTANT VELOCITY PHASE ----------
    elseif tk <= ta + T_const
        dt2    = tk - ta;
        q_rel  = 0.5*A*ta^2 + vmax*dt2;
        v_rel  = vmax;
        a_rel  = 0;

    % ---------- DECELERATION PHASE ----------
    else
        dt3    = tk - (ta + T_const);
        q_rel  = 0.5*A*ta^2 + vmax*T_const + (vmax*dt3 - 0.5*D*dt3^2);
        v_rel  = vmax - D*dt3;
        a_rel  = -D;
    end

    % apply direction of motion
    q(k)   = q0 + sgn * q_rel;
    qd(k)  = sgn * v_rel;
    qdd(k) = sgn * a_rel;
end

end
