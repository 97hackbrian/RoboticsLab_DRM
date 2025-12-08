% waypoint_navigation_simulation.m
% Simulación en lazo cerrado con navegación por waypoints discretos
% El robot se mueve de punto a punto en lugar de seguir trayectoria continua

% clc; close all; % Comentado para preservar workspace

% Agregar rutas necesarias
addpath('../config');
addpath('../system');
addpath('../controllers');

fprintf('\n╔════════════════════════════════════════════════════════════╗\n');
fprintf('║   NAVEGACIÓN POR WAYPOINTS - CONTROLADOR DIFUSO           ║\n');
fprintf('╚════════════════════════════════════════════════════════════╝\n\n');

%% 1. CARGAR PARÁMETROS
params = robot_parameters();

%% 2. CREAR SISTEMA DE INFERENCIA DIFUSO
fprintf('► Inicializando controlador difuso...\n');
fis = fuzzy_controller_setup();

%% 3. GENERAR WAYPOINTS
fprintf('\n► Generando waypoints...\n');

% Configuración de trayectoria
trajectory_type = 'circle'; % Opciones: 'line', 'square', 'circle', 's_curve'

switch trajectory_type
    case 'square'
        wp_params.side = 4; % metros
    case 'circle'
        wp_params.radius = 200;
        wp_params.num_points = 12; % 12 puntos alrededor del círculo
    case 's_curve'
        wp_params.num_points = 10; % 10 puntos para S-curve
    otherwise
        wp_params = struct();
end

waypoints = generate_waypoints(trajectory_type, wp_params);
N_waypoints = size(waypoints, 1);

fprintf('  Total waypoints: %d\n', N_waypoints);
for i = 1:min(N_waypoints, 5) % Mostrar solo primeros 5
    fprintf('    WP%d: (%.1f, %.1f)\n', i, waypoints(i,1), waypoints(i,2));
end
if N_waypoints > 5
    fprintf('    ...\n');
end

%% 4. CONDICIONES INICIALES
theta_0 = deg2rad(0.5); % Terreno casi plano
X0 = [0; 0; 0;  0; theta_0; 0;  0; 0; 0;  0; 0; 0];

fprintf('\n  Condiciones iniciales:\n');
fprintf('    Posición: (%.1f, %.1f) m\n', X0(1), X0(2));
fprintf('    Pitch: %.1f°\n\n', rad2deg(theta_0));

%% 5. PARÁMETROS DE NAVEGACIÓN
waypoint_threshold = 0.5; % Distancia para considerar waypoint alcanzado (metros)
dt_control = 0.01; % 100 Hz
max_time = 60; % Tiempo máximo de simulación (s)

current_waypoint_idx = 1;
target_waypoint = waypoints(current_waypoint_idx, :);

%% 6. SIMULACIÓN POR WAYPOINTS
fprintf('► Ejecutando navegación por waypoints...\n');

% Almacenamiento
max_steps = max_time / dt_control;
X_history = zeros(max_steps, 12);
U_history = zeros(max_steps, 4);
t_history = zeros(max_steps, 1);
waypoint_history = zeros(max_steps, 1);

X_current = X0;
X_ref_prev = [target_waypoint(1); target_waypoint(2); zeros(10,1)];
step = 1;
t_current = 0;
simulation_complete = false;

while ~simulation_complete && step < max_steps
    % Estado actual
    X_history(step,:) = X_current';
    t_history(step) = t_current;
    waypoint_history(step) = current_waypoint_idx;
    
    % Waypoint objetivo actual
    X_ref = [target_waypoint(1); target_waypoint(2); zeros(10,1)];
    
    % Calcular distancia al waypoint
    dist_to_waypoint = sqrt((X_current(1) - target_waypoint(1))^2 + ...
        (X_current(2) - target_waypoint(2))^2);
    
    % Verificar si alcanzamos el waypoint
    if dist_to_waypoint < waypoint_threshold
        fprintf('  ✓ Waypoint %d alcanzado (t=%.1fs, dist=%.2fm)\n', ...
            current_waypoint_idx, t_current, dist_to_waypoint);
        
        % Avanzar al siguiente waypoint
        current_waypoint_idx = current_waypoint_idx + 1;
        
        if current_waypoint_idx > N_waypoints
            fprintf('\n► Todos los waypoints alcanzados!\n');
            simulation_complete = true;
            break;
        end
        
        target_waypoint = waypoints(current_waypoint_idx, :);
        fprintf('    → Nuevo objetivo: WP%d (%.1f, %.1f)\n', ...
            current_waypoint_idx, target_waypoint(1), target_waypoint(2));
    end
    
    % Calcular control
    u_control = fuzzy_position_controller(X_current, X_ref, X_ref_prev, ...
        dt_control, fis, params);
    U_history(step,:) = u_control';
    
    % Integrar dinámica
    t_span = [t_current, t_current + dt_control];
    model_func = @(t,x) skid_steer_robot_model(t, x, u_control, params);
    opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
    [~, X_step] = ode45(model_func, t_span, X_current, opts);
    
    % Actualizar
    X_current = X_step(end,:)';
    X_ref_prev = X_ref;
    t_current = t_current + dt_control;
    step = step + 1;
    
    % Progreso
    if mod(step, 500) == 0
        fprintf('  t=%.1fs, WP%d, dist=%.2fm\n', t_current, current_waypoint_idx, dist_to_waypoint);
    end
end

% Recortar arrays
X_history = X_history(1:step-1, :);
U_history = U_history(1:step-1, :);
t_history = t_history(1:step-1);
waypoint_history = waypoint_history(1:step-1);

%% 7. ANÁLISIS
fprintf('\n► Análisis de resultados:\n');
fprintf('  Tiempo total: %.1f s\n', t_current);
fprintf('  Waypoints alcanzados: %d / %d\n', current_waypoint_idx - 1, N_waypoints);
fprintf('  Torque máximo: %.2f Nm\n', max(abs(U_history(:))));

%% 8. VISUALIZACIÓN
figure('Name', 'Navegación por Waypoints', 'Position', [50 50 1400 800]);

% Trayectoria XY
subplot(2,2,1);
plot(waypoints(:,1), waypoints(:,2), 'r--o', 'LineWidth', 2, 'MarkerSize', 10); hold on;
plot(X_history(:,1), X_history(:,2), 'b-', 'LineWidth', 1.5);
plot(X_history(1,1), X_history(1,2), 'go', 'MarkerSize', 12, 'LineWidth', 2);
plot(X_history(end,1), X_history(end,2), 'rs', 'MarkerSize', 12, 'LineWidth',2);
xlabel('X (m)'); ylabel('Y (m)');
title('Trayectoria XY');
legend('Waypoints', 'Trayectoria Real', 'Inicio', 'Fin', 'Location', 'best');
grid on; axis equal;

% Waypoint activo vs tiempo
subplot(2,2,2);
stairs(t_history, waypoint_history, 'LineWidth', 2);
xlabel('Tiempo (s)'); ylabel('Waypoint Activo');
title('Progreso de Waypoints');
grid on;

% Torques
subplot(2,2,3);
t_plot = t_history(1:size(U_history,1));
plot(t_plot, U_history(:,1:2), 'LineWidth', 1.5);
xlabel('Tiempo (s)'); ylabel('Torque (Nm)');
title('Torques de Control');
legend('FL/RL', 'FR/RR');
grid on;

% Velocidad
subplot(2,2,4);
plot(t_history, X_history(:,7), 'k', 'LineWidth', 1.5);
xlabel('Tiempo (s)'); ylabel('Velocidad (m/s)');
title('Velocidad Longitudinal');
grid on;

fprintf('\n════════════════════════════════════════════════════════\n');
fprintf('NAVEGACIÓN POR WAYPOINTS COMPLETADA\n');
fprintf('════════════════════════════════════════════════════════\n\n');
