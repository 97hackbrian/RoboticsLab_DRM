% rrt_star_navigation_simulation.m
% Simulación en TIEMPO REAL: RRT* Path Planning + Navegación con Controlador Fuzzy+PID+Lag
%
% Este script integra el algoritmo RRT* de path planning con el controlador
% de navegación existente en Control_lab/final_work
% La visualización es en tiempo real como realtime_waypoint_click_simulation.m

function rrt_star_navigation_simulation(scenario)
close all; clc;

%% ════════════════════════════════════════════════════════════════════════
%  CONFIGURACIÓN DE PATHS (Usar controlador existente)
%% ════════════════════════════════════════════════════════════════════════

% Path al controlador existente en Control_lab
% Subir: simulation -> final_work -> Perception_and_planning_lab -> RoboticsLab_DRM
repo_root = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
control_final_work = fullfile(repo_root, 'Control_lab', 'final_work');

% Agregar paths del sistema de control existente
addpath(fullfile(control_final_work, 'config'));
addpath(fullfile(control_final_work, 'controllers'));
addpath(fullfile(control_final_work, 'system'));

% Agregar paths locales del path planner
addpath('../config');
addpath('../planners');
addpath('../utils');

fprintf('\n╔════════════════════════════════════════════════════════════════╗\n');
fprintf('║   SIMULACIÓN RRT* EN TIEMPO REAL                              ║\n');
fprintf('║   Path Planning + Controlador Fuzzy+PID+Lag                   ║\n');
fprintf('╚════════════════════════════════════════════════════════════════╝\n\n');

%% ════════════════════════════════════════════════════════════════════════
%  1. CARGAR CONFIGURACIONES
%% ════════════════════════════════════════════════════════════════════════

if nargin < 1
    scenario = 'challenge';  % Escenario por defecto
end

% Cargar mapa del entorno
map = environment_map(scenario);

% Cargar parámetros del planificador RRT*
planner_params = planner_parameters();

% Cargar parámetros del robot (del sistema de control existente)
robot_params = robot_parameters();

% Inicializar controlador fuzzy (del sistema de control existente)
fprintf('► Cargando controlador Fuzzy+PID+Lag...\n');
fis = fuzzy_yaw_rate_controller_setup();

%% ════════════════════════════════════════════════════════════════════════
%  2. GENERAR PATH CON RRT*
%% ════════════════════════════════════════════════════════════════════════

fprintf('\n════════════════════════════════════════════════════════════════\n');
fprintf('                  FASE 1: PATH PLANNING (RRT*)                   \n');
fprintf('════════════════════════════════════════════════════════════════\n');

waypoints = generate_waypoints(map.start, map.goal, map, planner_params);

if isempty(waypoints)
    error('No se pudo generar un path válido. Intente con otro escenario.');
end

fprintf('► Waypoints generados: %d\n', size(waypoints, 1));
fprintf('► Presione cualquier tecla para iniciar navegación en tiempo real...\n');
pause;

% Cerrar figuras del RRT*
close all;

%% ════════════════════════════════════════════════════════════════════════
%  3. CONFIGURAR VISUALIZACIÓN EN TIEMPO REAL
%% ════════════════════════════════════════════════════════════════════════

fprintf('\n════════════════════════════════════════════════════════════════\n');
fprintf('              FASE 2: NAVEGACIÓN EN TIEMPO REAL                  \n');
fprintf('════════════════════════════════════════════════════════════════\n');
fprintf('► Instrucciones:\n');
fprintf('   - Observe el robot navegando en tiempo real\n');
fprintf('   - Cierre la ventana para terminar\n\n');

% Estado inicial del robot
X = zeros(12, 1);
X(1) = map.start(1);
X(2) = map.start(2);
X(6) = atan2(waypoints(2,2) - waypoints(1,2), waypoints(2,1) - waypoints(1,1));

% Parámetros de simulación
dt = 0.01;  % Paso de simulación
time_multiplier = 1.0;  % Factor de velocidad (1.0 = tiempo real)
waypoint_tolerance = planner_params.waypoint_tolerance;

% Índice del waypoint actual
current_waypoint_idx = 2;

% Variables de control
X_ref_prev = [waypoints(current_waypoint_idx, :)'; zeros(10, 1)];

%% Crear figura principal
fig = figure('Name', 'RRT* Real-Time Navigation', 'NumberTitle', 'off', ...
    'Position', [100 100 1000 800], 'Color', 'w');

ax = axes(fig);
hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)');
title(ax, 'Navegación RRT* en Tiempo Real');
xlim(ax, [map.x_min map.x_max]);
ylim(ax, [map.y_min map.y_max]);

% Dibujar obstáculos
for i = 1:length(map.obstacles)
    obs = map.obstacles{i};
    switch lower(obs.type)
        case 'circle'
            theta = linspace(0, 2*pi, 50);
            x_obs = obs.center(1) + obs.radius * cos(theta);
            y_obs = obs.center(2) + obs.radius * sin(theta);
            fill(ax, x_obs, y_obs, [0.4 0.4 0.4], 'FaceAlpha', 0.6, 'EdgeColor', 'k', 'LineWidth', 1.5);
        case 'rectangle'
            hw = obs.width/2; hh = obs.height/2;
            x_obs = obs.center(1) + [-hw hw hw -hw];
            y_obs = obs.center(2) + [-hh -hh hh hh];
            fill(ax, x_obs, y_obs, [0.4 0.4 0.4], 'FaceAlpha', 0.6, 'EdgeColor', 'k', 'LineWidth', 1.5);
    end
end

% Dibujar path planificado
plot(ax, waypoints(:,1), waypoints(:,2), 'b--', 'LineWidth', 1.5);
for i = 1:size(waypoints, 1)
    plot(ax, waypoints(i,1), waypoints(i,2), 'bo', 'MarkerSize', 10, 'LineWidth', 1.5);
    text(ax, waypoints(i,1)+0.3, waypoints(i,2)+0.3, num2str(i), 'FontSize', 8, 'Color', 'b');
end

% Marcar inicio y meta
plot(ax, map.start(1), map.start(2), 'go', 'MarkerSize', 18, 'LineWidth', 3);
text(ax, map.start(1), map.start(2)-1.2, 'INICIO', 'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'Color', 'g');
plot(ax, map.goal(1), map.goal(2), 'r*', 'MarkerSize', 18, 'LineWidth', 3);
text(ax, map.goal(1), map.goal(2)+1.2, 'META', 'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'Color', 'r');

% Elementos dinámicos del robot
robot_body = patch(ax, 'XData', [], 'YData', [], ...
    'FaceColor', 'b', 'FaceAlpha', 0.3, 'EdgeColor', 'k', 'LineWidth', 2);
heading_arrow = quiver(ax, 0, 0, 1, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'AutoScale', 'off');

% Trayectoria del robot (estela)
trail_plot = plot(ax, 0, 0, 'g-', 'LineWidth', 2);
X_trail = []; Y_trail = [];

% Marcador del waypoint objetivo actual
target_marker = plot(ax, waypoints(current_waypoint_idx, 1), ...
    waypoints(current_waypoint_idx, 2), 'mx', 'MarkerSize', 25, 'LineWidth', 4);
target_circle = plot(ax, waypoints(current_waypoint_idx, 1), ...
    waypoints(current_waypoint_idx, 2), 'mo', 'MarkerSize', 30, 'LineWidth', 2);

% Texto de estado
status_text = text(ax, map.x_min + 1, map.y_max - 1.5, 'Iniciando...', ...
    'FontSize', 11, 'BackgroundColor', 'w', 'EdgeColor', 'k', 'Margin', 3);

%% ════════════════════════════════════════════════════════════════════════
%  4. BUCLE DE SIMULACIÓN EN TIEMPO REAL
%% ════════════════════════════════════════════════════════════════════════

t_current = 0;
last_tic = tic;
navigation_complete = false;

try
    while isvalid(fig) && ~navigation_complete
        % Gestionar tiempo real
        dt_real = toc(last_tic);
        last_tic = tic;
        
        % Objetivo actual
        current_target = waypoints(current_waypoint_idx, :)';
        
        % Actualizar marcador visual del objetivo
        set(target_marker, 'XData', current_target(1), 'YData', current_target(2));
        set(target_circle, 'XData', current_target(1), 'YData', current_target(2));
        
        % Calcular distancia al waypoint
        dist_to_waypoint = norm(X(1:2) - current_target);
        v_abs = abs(X(7));
        
        % Verificar llegada al waypoint
        if dist_to_waypoint < waypoint_tolerance
            fprintf('  ✓ Waypoint %d alcanzado en t=%.2fs\n', current_waypoint_idx, t_current);
            set(target_circle, 'Color', 'g');
            
            if current_waypoint_idx >= size(waypoints, 1)
                navigation_complete = true;
                fprintf('\n✓✓✓ META ALCANZADA en t=%.2fs ✓✓✓\n', t_current);
                set(status_text, 'String', sprintf('¡META ALCANZADA! T=%.1fs', t_current), ...
                    'BackgroundColor', 'g');
                break;
            end
            
            current_waypoint_idx = current_waypoint_idx + 1;
            current_target = waypoints(current_waypoint_idx, :)';
            set(target_circle, 'Color', 'm');
        end
        
        % Construir referencia
        X_ref = zeros(12, 1);
        X_ref(1) = current_target(1);
        X_ref(2) = current_target(2);
        
        % Calcular control (usando controlador existente)
        torques = fuzzy_yaw_rate_controller(X, X_ref, X_ref_prev, dt, fis, robot_params);
        
        % Integrar dinámica
        model_func = @(t, x) skid_steer_robot_model(t, x, torques, robot_params);
        [~, X_ode] = ode45(model_func, [0 dt], X);
        X = X_ode(end, :)';
        
        % Guardar estela
        X_trail = [X_trail, X(1)];
        Y_trail = [Y_trail, X(2)];
        if length(X_trail) > 1000
            X_trail = X_trail(end-499:end);
            Y_trail = Y_trail(end-499:end);
        end
        set(trail_plot, 'XData', X_trail, 'YData', Y_trail);
        
        % Actualizar gráficos del robot
        update_robot_graphics_2d(X, robot_params, robot_body, heading_arrow);
        
        % Actualizar texto de estado
        status_str = sprintf('T: %.1fs | WP: %d/%d | Dist: %.2fm | V: %.2fm/s', ...
            t_current, current_waypoint_idx, size(waypoints, 1), dist_to_waypoint, v_abs);
        set(status_text, 'String', status_str);
        
        % Actualizar pantalla
        drawnow limitrate;
        
        % Avanzar tiempo
        X_ref_prev = X_ref;
        t_current = t_current + dt;
    end
catch ME
    if strcmp(ME.identifier, 'MATLAB:class:InvalidHandle')
        fprintf('► Simulación terminada por el usuario.\n');
    else
        rethrow(ME);
    end
end

%% ════════════════════════════════════════════════════════════════════════
%  5. RESUMEN FINAL
%% ════════════════════════════════════════════════════════════════════════

fprintf('\n════════════════════════════════════════════════════════════════\n');
fprintf('                      RESUMEN DE NAVEGACIÓN                      \n');
fprintf('════════════════════════════════════════════════════════════════\n');
fprintf('  Tiempo total:          %.2f s\n', t_current);
fprintf('  Waypoints alcanzados:  %d / %d\n', ...
    min(current_waypoint_idx, size(waypoints, 1)), size(waypoints, 1));
fprintf('  Distancia recorrida:   %.2f m\n', compute_trail_length(X_trail, Y_trail));
if navigation_complete
    fprintf('  Estado final:          META ALCANZADA\n');
else
    fprintf('  Estado final:          EN PROGRESO\n');
end
fprintf('════════════════════════════════════════════════════════════════\n');

end

%% ════════════════════════════════════════════════════════════════════════
%  FUNCIONES AUXILIARES
%% ════════════════════════════════════════════════════════════════════════

function update_robot_graphics_2d(X, p, h_body, h_arrow)
x = X(1); y = X(2); psi = X(6);
L = p.L; W = p.W;

% Vértices del robot (rectángulo) - FL, FR, RR, RL
pts_local = [L/2, W/2; L/2, -W/2; -L/2, -W/2; -L/2, W/2]';

% Matriz de rotación 2D
R = [cos(psi), -sin(psi); sin(psi), cos(psi)];

% Transformar al mundo
pts_world = R * pts_local + [x; y];

% Actualizar Patch
set(h_body, 'XData', pts_world(1,:), 'YData', pts_world(2,:));

% Actualizar Flecha
set(h_arrow, 'XData', x, 'YData', y, ...
    'UData', cos(psi)*1.5, 'VData', sin(psi)*1.5);
end

function len = compute_trail_length(X_trail, Y_trail)
len = 0;
for i = 1:(length(X_trail)-1)
    len = len + sqrt((X_trail(i+1)-X_trail(i))^2 + (Y_trail(i+1)-Y_trail(i))^2);
end
end
