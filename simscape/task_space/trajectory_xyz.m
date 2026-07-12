function [x, y, z] = trajectory_xyz(t)

    % --- definizione durata traiettorie ---
    T1 = 2;   % durata tratto rettilineo
    T2 = 2;   % durata semi-circonferenza
    Ttot = T1 + T2;

    % --- parametri geometrici ---
    z0 = 285;
    x_start = 310;
    x_end   = 485;
    R = 200;

    % centro della semicirconferenza
    xc = x_end;
    zc = z0 + R;

    % --- tratto 1: linea ---
    if t <= T1
        s = t / T1;               % s ∈ [0,1]
        x = x_start + s * (x_end - x_start);
        z = z0;
        y = 0;
        return;
    end

    % --- tratto 2: semicirconferenza ---
    if t <= T1 + T2
        tau = t - T1;
        s = tau / T2;             % s ∈ [0,1]
        theta = -pi/2 + s*pi;     % da -90° a +90°

        x = xc + R * cos(theta);
        z = zc + R * sin(theta);
        y = 0;
        return;
    end

    % --- fine traiettoria: tieni ultimo punto ---
    x = xc;
    z = zc + R;
    y = 0;
end
