% closed_loop_simulation.m
% Simulación en lazo cerrado con controlador difuso de posición
% El controlador sigue una trayectoria de referencia

% clc; close all; % Comentado para preservar workspace

% Agregar rutas necesarias
addpath('../config');
addpath('../system');
addpath('../controllers');

fprintf('\n╔════════════════════════════════════════════════════════════╗\n');
fprintf('║   SIMULACIÓN LAZO CERRADO - CONTROLADOR DIFUSO            ║\n');
fprintf('╚════════════════════════════════════════════════════════════╝\n\n');

%% 1. CARGAR PARÁMETROS
params = robot_parameters();

%% 2. CREAR SISTEMA DE INFERENCIA DIFUSO
fprintf('► Inicializando controlador difuso...\n');
fis = fuzzy_controller_setup();

%% 3. GENERAR TRAYECTORIA DE REFERENCIA
fprintf('\n► Generando trayectoria de referencia...\n');

% Seleccionar tipo de trayectoria
% Opciones: 'line', 'circle', 's_curve', 'step', 'square'
trajectory_type = 's_curve';
t_sim = [0 10]; % Tiempo de simulación (segundos)
dt_control = 0.01; % Paso de control (100 Hz)

% Generar trayectoria
traj_params.radius = 2;  % Reducido de 3m a 2m para giros más suaves
traj_params.period = 12; % Aumentado de 8s a 12s para velocidad angular más baja
[t_ref, X_ref_traj] = trajectory_generator(trajectory_type, t_sim, dt_control, traj_params);

%% 4. CONDICIONES INICIALES
% Robot comienza en origen con terreno plano
theta_0 = deg2rad(0.5); % Pendiente mínima (casi plano)
X0 = [0; 0; 0;  0; theta_0; 0;  0; 0; 0;  0; 0; 0];

fprintf('  Condiciones iniciales:\n');
fprintf('    Posición: (%.1f, %.1f) m\n', X0(1), X0(2));
fprintf('    Pitch: %.1f°\n', rad2deg(theta_0));

%% 5. SIMULACIÓN EN LAZO CERRADO
fprintf('\n► Ejecutando simulación en lazo cerrado...\n');

% Preparar almacenamiento de resultados
N_steps = length(t_ref);
X_history = zeros(N_steps, 12);
U_history = zeros(N_steps, 4);
Error_history = zeros(N_steps, 2);

% Estado inicial
X_current = X0;
X_history(1,:) = X_current';

% Referencia anterior (para calcular derivada)
X_ref_prev = X_ref_traj(1,:)';

% Simulación paso a paso
for i = 1:N_steps-1
    % Obtener referencia actual
    X_ref_current = X_ref_traj(i,:)';
    
    % Calcular control usando controlador difuso
    u_control = fuzzy_position_controller(X_current, X_ref_current, ...
        X_ref_prev, dt_control, fis, params);
    
    % Guardar torques
    U_history(i,:) = u_control';
    
    % Guardar error
    Error_history(i,1) = X_ref_current(1) - X_current(1); % Error X
    Error_history(i,2) = X_ref_current(2) - X_current(2); % Error Y
    
    % Integrar dinámica del robot
    t_span_step = [t_ref(i), t_ref(i+1)];
    model_func = @(t,x) skid_steer_robot_model(t, x, u_control, params);
    
    % Integración numérica
    opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-8, 'MaxStep', dt_control);
    [~, X_step] = ode45(model_func, t_span_step, X_current, opts);
    
    % Actualizar estado
    X_current = X_step(end,:)';
    X_history(i+1,:) = X_current';
    
    % Actualizar referencia anterior
    X_ref_prev = X_ref_current;
    
    % Progreso cada 10%
    if mod(i, floor(N_steps/10)) == 0
        fprintf('  Progreso: %d%%\n', round(100*i/N_steps));
    end
end

% Último punto de error
Error_history(end,1) = X_ref_traj(end,1) - X_history(end,1);
Error_history(end,2) = X_ref_traj(end,2) - X_history(end,2);

fprintf('✓ Simulación completada\n\n');

%% 6. ANÁLISIS DE RESULTADOS
fprintf('► Análisis de resultados:\n');

% Error RMS
error_rms_x = sqrt(mean(Error_history(:,1).^2));
error_rms_y = sqrt(mean(Error_history(:,2).^2));
error_rms_total = sqrt(error_rms_x^2 + error_rms_y^2);

fprintf('  Error RMS X: %.3f m\n', error_rms_x);
fprintf('  Error RMS Y: %.3f m\n', error_rms_y);
fprintf('  Error RMS Total: %.3f m\n', error_rms_total);

% Error máximo
max_error = max(sqrt(Error_history(:,1).^2 + Error_history(:,2).^2));
fprintf('  Error Máximo: %.3f m\n', max_error);

% Estadísticas de torque
fprintf('  Torque promedio: %.2f Nm\n', mean(abs(U_history(:))));
fprintf('  Torque máximo: %.2f Nm\n', max(abs(U_history(:))));

%% 7. VISUALIZACIÓN
figure('Name', 'Tracking Control - Fuzzy Controller', 'Position', [50 50 1400 800]);

% Subplot 1: Trayectoria XY
subplot(2,3,1);
plot(X_ref_traj(:,1), X_ref_traj(:,2), 'r--', 'LineWidth', 2); hold on;
plot(X_history(:,1), X_history(:,2), 'b-', 'LineWidth', 1.5);
plot(X_history(1,1), X_history(1,2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(X_history(end,1), X_history(end,2), 'rs', 'MarkerSize', 10, 'LineWidth', 2);
xlabel('X (m)'); ylabel('Y (m)');
title('Trayectoria XY');
legend('Referencia', 'Real', 'Inicio', 'Fin', 'Location', 'best');
grid on; axis equal;

% Subplot 2: Error de posición X
subplot(2,3,2);
plot(t_ref, Error_history(:,1), 'r', 'LineWidth', 1.5);
xlabel('Tiempo (s)'); ylabel('Error X (m)');
title(sprintf('Error de Posición X (RMS=%.3f m)', error_rms_x));
grid on;

% Subplot 3: Error de posición Y
subplot(2,3,3);
plot(t_ref, Error_history(:,2), 'b', 'LineWidth', 1.5);
xlabel('Tiempo (s)'); ylabel('Error Y (m)');
title(sprintf('Error de Posición Y (RMS=%.3f m)', error_rms_y));
grid on;

% Subplot 4: Torques de control
subplot(2,3,4);
t_torque = t_ref(1:size(U_history,1)); % Match lengths
plot(t_torque, U_history(:,1), 'r', 'LineWidth', 1); hold on;
plot(t_torque, U_history(:,2), 'b', 'LineWidth', 1);
plot(t_torque, U_history(:,3), 'g', 'LineWidth', 1);
plot(t_torque, U_history(:,4), 'm', 'LineWidth', 1);
xlabel('Tiempo (s)'); ylabel('Torque (Nm)');
title('Torques de Control');
legend('FL', 'FR', 'RL', 'RR', 'Location', 'best');
grid on;

% Subplot 5: Velocidad longitudinal
subplot(2,3,5);
plot(t_ref, X_history(:,7), 'k', 'LineWidth', 1.5);
xlabel('Tiempo (s)'); ylabel('Velocidad (m/s)');
title('Velocidad Longitudinal');
grid on;

% Subplot 6: Ángulo Pitch
subplot(2,3,6);
plot(t_ref, rad2deg(X_history(:,5)), 'm', 'LineWidth', 1.5);
xlabel('Tiempo (s)'); ylabel('Pitch (°)');
title('Inclinación (Pitch)');
grid on;

%% 8. RESUMEN
fprintf('\n════════════════════════════════════════════════════════\n');
fprintf('RESUMEN DE SIMULACIÓN\n');
fprintf('════════════════════════════════════════════════════════\n');
fprintf('Trayectoria: %s\n', trajectory_type);
fprintf('Tiempo simulado: %.1f s\n', t_sim(2));
fprintf('Rendimiento de tracking:\n');
fprintf('  • Error RMS: %.3f m\n', error_rms_total);
fprintf('  • Error máximo: %.3f m\n', max_error);
fprintf('  • Torque máximo usado: %.2f Nm (límite: %.2f Nm)\n', ...
    max(abs(U_history(:))), params.tau_max);
fprintf('════════════════════════════════════════════════════════\n\n');
