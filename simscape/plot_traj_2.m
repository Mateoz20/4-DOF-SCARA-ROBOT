%% Estraggo tempo e posizione end-effector
t   = out.EE_pos.time;                  % tempo
pos = out.EE_pos.signals.values;        % [x y z] in metri

% Componenti
x = pos(:,1);
y = pos(:,2);
z = pos(:,3);

%% 1) TRAIETTORIA NEL PIANO X–Z
figure;
plot(x*1000, z*1000, 'LineWidth', 2);
xlabel('X [mm]', 'FontSize', 12);
ylabel('Z [mm]', 'FontSize', 12);
title('Traiettoria End-Effector – Piano XZ', 'FontSize', 14);
grid on; axis equal;

%% 2) TRAIETTORIA NEL PIANO X–Y
figure;
plot(x*1000, y*1000, 'LineWidth', 2);
xlabel('X [mm]', 'FontSize', 12);
ylabel('Y [mm]', 'FontSize', 12);
title('Traiettoria End-Effector – Piano XY', 'FontSize', 14);
grid on; axis equal;

%% 3) TRAIETTORIA 3D
figure;
plot3(x*1000, y*1000, z*1000, 'LineWidth', 2);
xlabel('X [mm]', 'FontSize', 12);
ylabel('Y [mm]', 'FontSize', 12);
zlabel('Z [mm]', 'FontSize', 12);
title('Traiettoria End-Effector – 3D', 'FontSize', 14);
grid on; axis equal;
view(45,25);   % angolo bello per visualizzare
