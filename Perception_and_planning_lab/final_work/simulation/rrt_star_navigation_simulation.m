% rrt_star_navigation_simulation.m
% Simulación completa: RRT* Path Planning + Navegación con Controlador Fuzzy+PID+Lag
%
% Este script integra el algoritmo RRT* de path planning con el controlador
% de navegación existente en Control_lab/final_work

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
fprintf('║   SIMULACIÓN RRT* + CONTROLADOR FUZZY+PID+LAG                ║\n');
fprintf('║   Path Planning con Navegación Autónoma                       ║\n');
fprintf('╚════════════════════════════════════════════════════════════════╝\n\n');

%% ════════════════════════════════════════════════════════════════════════
%  1. CARGAR CONFIGURACIONES
%% ════════════════════════════════════════════════════════════════════════

if nargin < 1
    scenario = 'simple';  % Escenario por defecto
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
fprintf('► Presione cualquier tecla para iniciar navegación...\n');
pause;

% Cerrar figuras del RRT* para evitar conflictos
close all;

%% ════════════════════════════════════════════════════════════════════════
%  3. NAVEGACIÓN CON CONTROLADOR
%% ════════════════════════════════════════════════════════════════════════

fprintf('\n════════════════════════════════════════════════════════════════\n');
fprintf('                 FASE 2: NAVEGACIÓN AUTÓNOMA                     \n');
fprintf('════════════════════════════════════════════════════════════════\n');

% Estado inicial del robot
X = zeros(12, 1);
X(1) = map.start(1);  % Posición X inicial
X(2) = map.start(2);  % Posición Y inicial
X(6) = atan2(waypoints(2,2) - waypoints(1,2), ...
    waypoints(2,1) - waypoints(1,1));  % Orientación hacia primer waypoint

% Parámetros de simulación
dt = 0.01;
max_time = 120;  % segundos máximo
waypoint_tolerance = planner_params.waypoint_tolerance;

% Índice del waypoint actual
current_waypoint_idx = 2;  % Empezar desde el segundo (primero es start)

% Variables de control
X_ref_prev = [waypoints(current_waypoint_idx, :)'; zeros(10, 1)];

%% Configurar visualización
fig = figure('Name', 'RRT* Navigation Simulation', 'Position', [50 50 1200 800]);

% Subplot 1: Vista 2D de navegación
ax_nav = subplot(1, 2, 1);
hold(ax_nav, 'on'); grid(ax_nav, 'on'); axis(ax_nav, 'equal');
xlabel(ax_nav, 'X (m)'); ylabel(ax_nav, 'Y (m)');
title(ax_nav, 'Navegación en Tiempo Real');
xlim(ax_nav, [map.x_min map.x_max]);
ylim(ax_nav, [map.y_min map.y_max]);

% Dibujar obstáculos
draw_obstacles_on_ax(ax_nav, map.obstacles);

% Dibujar waypoints planificados
plot(ax_nav, waypoints(:,1), waypoints(:,2), 'b--', 'LineWidth', 1.5);
plot(ax_nav, waypoints(:,1), waypoints(:,2), 'bo', 'MarkerSize', 8);

% Marcar inicio y meta
plot(ax_nav, map.start(1), map.start(2), 'go', 'MarkerSize', 15, 'LineWidth', 3);
plot(ax_nav, map.goal(1), map.goal(2), 'r*', 'MarkerSize', 15, 'LineWidth', 3);

% Elementos dinámicos
robot_body = patch(ax_nav, 'XData', [], 'YData', [], ...
    'FaceColor', [0 0.5 1], 'FaceAlpha', 0.5, 'EdgeColor', 'k', 'LineWidth', 2);
heading_arrow = quiver(ax_nav, 0, 0, 1, 0, 'r', 'LineWidth', 2, ...
    'MaxHeadSize', 0.8, 'AutoScale', 'off');
trail_plot = plot(ax_nav, 0, 0, 'g-', 'LineWidth', 2);
current_target_marker = plot(ax_nav, waypoints(current_waypoint_idx, 1), ...
    waypoints(current_waypoint_idx, 2), 'mx', 'MarkerSize', 20, 'LineWidth', 3);

% Subplot 2: Métricas en tiempo real
ax_metrics = subplot(1, 2, 2);
hold(ax_metrics, 'on'); grid(ax_metrics, 'on');
xlabel(ax_metrics, 'Tiempo (s)');

% Variables para gráficas de métricas
t_history = [];
dist_history = [];
vel_history = [];
omega_history = [];

% Texto de estado
status_text = text(ax_nav, map.x_min + 1, map.y_max - 1, '', ...
    'FontSize', 10, 'BackgroundColor', 'w', 'EdgeColor', 'k');

%% Bucle de simulación
X_trail = []; Y_trail = [];
t_current = 0;
step = 0;
navigation_complete = false;

fprintf('► Iniciando navegación...\n');

while t_current < max_time && isvalid(fig) && ~navigation_complete
    step = step + 1;
    
    %% Obtener objetivo actual
    current_target = waypoints(current_waypoint_idx, :)';
    
    %% Calcular distancia al waypoint actual
    dist_to_waypoint = norm(X(1:2) - current_target);
    
    %% Verificar si llegamos al waypoint actual
    if dist_to_waypoint < waypoint_tolerance
        fprintf('  ✓ Waypoint %d alcanzado en t=%.2fs\n', current_waypoint_idx, t_current);
        
        if current_waypoint_idx >= size(waypoints, 1)
            navigation_complete = true;
            fprintf('\n✓✓✓ META ALCANZADA ✓✓✓\n');
            break;
        end
        
        current_waypoint_idx = current_waypoint_idx + 1;
        current_target = waypoints(current_waypoint_idx, :)';
        
        % Actualizar marcador de objetivo
        if isvalid(current_target_marker)
            set(current_target_marker, 'XData', current_target(1), 'YData', current_target(2));
        end
    end
    
    %% Construir referencia para controlador
    X_ref = zeros(12, 1);
    X_ref(1) = current_target(1);
    X_ref(2) = current_target(2);
    
    %% Calcular control (usando controlador existente)
    torques = fuzzy_yaw_rate_controller(X, X_ref, X_ref_prev, dt, fis, robot_params);
    
    %% Integrar dinámica del robot (usando modelo existente)
    model_func = @(t, x) skid_steer_robot_model(t, x, torques, robot_params);
    [~, X_ode] = ode45(model_func, [0 dt], X);
    X = X_ode(end, :)';
    
    %% Guardar trayectoria
    X_trail = [X_trail, X(1)];
    Y_trail = [Y_trail, X(2)];
    
    % Limitar longitud del trail
    if length(X_trail) > 2000
        X_trail = X_trail(end-999:end);
        Y_trail = Y_trail(end-999:end);
    end
    
    %% Guardar métricas
    t_history = [t_history, t_current];
    dist_history = [dist_history, dist_to_waypoint];
    vel_history = [vel_history, X(7)];
    omega_history = [omega_history, X(12)];
    
    %% Actualizar visualización (cada 5 pasos)
    if mod(step, 5) == 0 && isvalid(robot_body)
        % Actualizar robot
        update_robot_graphics(X, robot_params, robot_body, heading_arrow);
        
        % Actualizar trail
        if isvalid(trail_plot)
            set(trail_plot, 'XData', X_trail, 'YData', Y_trail);
        end
        
        % Actualizar texto de estado
        status_str = sprintf('T: %.1fs | WP: %d/%d | Dist: %.2fm | V: %.2fm/s', ...
            t_current, current_waypoint_idx, size(waypoints, 1), ...
            dist_to_waypoint, abs(X(7)));
        if isvalid(status_text)
            set(status_text, 'String', status_str);
        end
        
        % Actualizar gráfica de métricas
        if mod(step, 50) == 0
            cla(ax_metrics);
            
            subplot(3, 1, 1);
            plot(t_history, dist_history, 'b-', 'LineWidth', 1.5);
            ylabel('Distancia (m)');
            title('Métricas de Navegación');
            grid on;
            
            subplot(3, 1, 2);
            plot(t_history, vel_history, 'g-', 'LineWidth', 1.5);
            ylabel('Velocidad (m/s)');
            grid on;
            
            subplot(3, 1, 3);
            plot(t_history, omega_history, 'r-', 'LineWidth', 1.5);
            ylabel('Omega (rad/s)');
            xlabel('Tiempo (s)');
            grid on;
        end
        
        drawnow limitrate;
    end
    
    %% Avanzar tiempo
    X_ref_prev = X_ref;
    t_current = t_current + dt;
end

%% ════════════════════════════════════════════════════════════════════════
%  4. RESUMEN FINAL
%% ════════════════════════════════════════════════════════════════════════

fprintf('\n════════════════════════════════════════════════════════════════\n');
fprintf('                      RESUMEN DE NAVEGACIÓN                      \n');
fprintf('════════════════════════════════════════════════════════════════\n');
fprintf('  Tiempo total:          %.2f s\n', t_current);
fprintf('  Waypoints alcanzados:  %d / %d\n', ...
    min(current_waypoint_idx, size(waypoints, 1)), size(waypoints, 1));
fprintf('  Distancia recorrida:   %.2f m\n', compute_trail_length(X_trail, Y_trail));
fprintf('  Estado final:          %s\n', conditional_str(navigation_complete, 'META ALCANZADA', 'TIEMPO AGOTADO'));
fprintf('════════════════════════════════════════════════════════════════\n');

end

%% ════════════════════════════════════════════════════════════════════════
%  FUNCIONES AUXILIARES
%% ════════════════════════════════════════════════════════════════════════

function draw_obstacles_on_ax(ax, obstacles)
for i = 1:length(obstacles)
    obs = obstacles{i};
    switch lower(obs.type)
        case 'circle'
            theta = linspace(0, 2*pi, 50);
            x = obs.center(1) + obs.radius * cos(theta);
            y = obs.center(2) + obs.radius * sin(theta);
            fill(ax, x, y, [0.4 0.4 0.4], 'FaceAlpha', 0.6, 'EdgeColor', 'k');
        case 'rectangle'
            hw = obs.width/2; hh = obs.height/2;
            x = obs.center(1) + [-hw hw hw -hw];
            y = obs.center(2) + [-hh -hh hh hh];
            fill(ax, x, y, [0.4 0.4 0.4], 'FaceAlpha', 0.6, 'EdgeColor', 'k');
    end
end
end

function update_robot_graphics(X, p, h_body, h_arrow)
x = X(1); y = X(2); psi = X(6);
L = p.L; W = p.W;

% Vértices del robot (rectángulo)
pts_local = [L/2, W/2; L/2, -W/2; -L/2, -W/2; -L/2, W/2]';

% Rotación
R = [cos(psi), -sin(psi); sin(psi), cos(psi)];
pts_world = R * pts_local + [x; y];

% Actualizar gráficos
set(h_body, 'XData', pts_world(1,:), 'YData', pts_world(2,:));
set(h_arrow, 'XData', x, 'YData', y, ...
    'UData', cos(psi)*1.2, 'VData', sin(psi)*1.2);
end

function len = compute_trail_length(X_trail, Y_trail)
len = 0;
for i = 1:(length(X_trail)-1)
    len = len + sqrt((X_trail(i+1)-X_trail(i))^2 + (Y_trail(i+1)-Y_trail(i))^2);
end
end

function s = conditional_str(cond, true_str, false_str)
if cond
    s = true_str;
else
    s = false_str;
end
end
