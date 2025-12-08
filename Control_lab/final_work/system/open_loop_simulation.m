% open_loop_simulation.m
% Simulación en lazo abierto del robot Skid-Steer en pendiente
% Muestra el comportamiento del sistema sin controlador (solo torque constante)

% clear; clc; close all; % Commented to preserve workspace when called from run_system.m
clc; close all;

% Agregar rutas necesarias
addpath('../config');
addpath('../system');

%% 1. CARGAR PARÁMETROS
params = robot_parameters();

%% 2. CONDICIONES INICIALES
% Robot en terreno con pendiente de 5 grados
% NOTA: En lazo abierto, pendientes grandes causan inestabilidad
theta_0 = deg2rad(5);
X0 = [0; 0; 0;  0; theta_0; 0;  0; 0; 0;  0; 0; 0];

%% 3. DEFINIR ENTRADA DE CONTROL
% Torque constante en las 4 ruedas (suficiente para vencer gravedad)
% Torque mínimo teórico para 5°: (20*9.81*sin(5°)*0.1)/4 ≈ 0.43 Nm
% Usamos 3 Nm para tener margen
u_control = [3; 3; 3; 3]; % Nm

%% 4. SIMULACIÓN
t_span = [0 3]; % 3 segundos (reducido para evitar divergencia)

% Función wrapper para pasar u y params
model_func = @(t,x) skid_steer_robot_model(t, x, u_control, params);

% Opciones de ODE para evitar inestabilidad numérica
opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-8, 'MaxStep', 0.01);
[t, X_real] = ode45(model_func, t_span, X0, opts);

%% 5. GENERACIÓN DE SENSORES (Modelo de Medición + Incertidumbre)
% Aquí simulamos lo que el sistema de tracking leerá realmente

% --- A. Encoder (Odometría ruidosa) ---
% Simulamos encoders leyendo velocidad rueda + resbalamiento aleatorio
noise_enc = params.noise_enc;
encoders_v = X_real(:,7) + (randn(size(X_real(:,7))) * noise_enc);
% Nota: En skid steer, v_encoder != v_real debido al slip geometry

% --- B. IMU (Acelerómetro y Giroscopio) ---
bias_gyro = params.bias_gyro(1);
noise_gyro = params.noise_gyro;
imu_gyro = X_real(:, 10:12) + bias_gyro + (randn(size(X_real(:, 10:12))) * noise_gyro);

% Acelerómetro (Mide Fuerza Específica = Accel - Gravedad_Body)
g_vec = 9.81;
imu_accel = zeros(length(t), 3);
for i=1:length(t)
    phi=X_real(i,4); theta=X_real(i,5);
    % Gravedad proyectada en cuerpo
    g_body = [-g_vec*sin(theta); g_vec*cos(theta)*sin(phi); g_vec*cos(theta)*cos(phi)];
    imu_accel(i,:) = (g_body)' + (randn(1,3)*params.noise_accel);
end

% --- C. Visual Pose (Camera) ---
% Baja frecuencia y ruido gaussiano en posición
noise_cam = params.noise_cam;
cam_pose = X_real(:, 1:3) + (randn(size(X_real(:, 1:3))) * noise_cam);

%% 6. VISUALIZACIÓN
figure('Name', 'Simulación en Lazo Abierto', 'Position', [100 100 1200 800]);

subplot(3,2,1);
plot(t, X_real(:,1), 'k', 'LineWidth', 2); hold on;
plot(t, cam_pose(:,1), 'r.');
title('Posición X (Real vs Medición Cámara)');
xlabel('Tiempo (s)'); ylabel('X (m)');
legend('Real', 'Cámara'); grid on;

subplot(3,2,2);
plot(t, X_real(:,2), 'k', 'LineWidth', 2); hold on;
plot(t, cam_pose(:,2), 'r.');
title('Posición Y');
xlabel('Tiempo (s)'); ylabel('Y (m)');
legend('Real', 'Cámara'); grid on;

subplot(3,2,3);
plot(t, X_real(:,7), 'b', 'LineWidth', 2); hold on;
plot(t, encoders_v, 'g--');
title('Velocidad Longitudinal (Real vs Encoder)');
xlabel('Tiempo (s)'); ylabel('u (m/s)');
legend('Real', 'Encoder'); grid on;

subplot(3,2,4);
plot(t, rad2deg(X_real(:,5)), 'm', 'LineWidth', 2);
title('Inclinación (Pitch) en Pendiente');
xlabel('Tiempo (s)'); ylabel('Pitch (°)');
grid on;

subplot(3,2,5);
plot(t, rad2deg(X_real(:,6)), 'c', 'LineWidth', 2);
title('Orientación (Yaw)');
xlabel('Tiempo (s)'); ylabel('Yaw (°)');
grid on;

subplot(3,2,6);
plot(t, X_real(:,10), 'r', t, X_real(:,11), 'g', t, X_real(:,12), 'b', 'LineWidth', 1.5);
title('Velocidades Angulares');
xlabel('Tiempo (s)'); ylabel('rad/s');
legend('p (roll rate)', 'q (pitch rate)', 'r (yaw rate)');
grid on;

%% 7. RESUMEN
fprintf('\n========== SIMULACIÓN EN LAZO ABIERTO ==========\n');
fprintf('Duración: %.1f segundos\n', t_span(2));
fprintf('Pendiente inicial: %.1f grados\n', rad2deg(theta_0));
fprintf('Torque aplicado: %.1f Nm por rueda\n', u_control(1));
fprintf('\nResultados finales:\n');
fprintf('  Posición X final: %.2f m\n', X_real(end,1));
fprintf('  Posición Y final: %.2f m\n', X_real(end,2));
fprintf('  Velocidad final: %.2f m/s\n', X_real(end,7));
fprintf('  Pitch final: %.2f°\n', rad2deg(X_real(end,5)));
fprintf('================================================\n\n');
