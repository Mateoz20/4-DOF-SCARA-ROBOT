function T_rise = min_rise_time_custom(motor, dq)
% dq = spostamento del giunto
% metodo identico a quello delle slide del prof

T_lim = motor.V*(1/motor.A + 1/motor.D);
dq_lim = 0.5*motor.V^2*(1/motor.A + 1/motor.D);

if dq < dq_lim
    % case 1
    T_rise = (sqrt(motor.A/motor.D) + sqrt(motor.D/motor.A)) * ...
             sqrt( 2*dq / (motor.A + motor.D) );
else
    % case 2
    T_rise = dq/motor.V + motor.V*(motor.A + motor.D)/(2*motor.A*motor.D);
end
end
