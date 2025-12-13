% test_full_navigation.m
% Test de integración: RRT* + Controlador Fuzzy+PID+Lag
% Verifica que el robot sigue correctamente los waypoints generados

function test_full_navigation()
close all; clc;

%% ════════════════════════════════════════════════════════════════════════
%  CONFIGURACIÓN DE PATHS
%% ════════════════════════════════════════════════════════════════════════

% Path al controlador existente
% Subir: tests -> final_work -> Perception_and_planning_lab -> RoboticsLab_DRM
repo_root = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));

control_final_work = fullfile(repo_root, 'Control_lab', 'final_work');

addpath(fullfile(control_final_work, 'config'));
addpath(fullfile(control_final_work, 'controllers'));
addpath(fullfile(control_final_work, 'system'));

addpath('../config');
addpath('../planners');
addpath('../utils');

fprintf('\n╔════════════════════════════════════════════════════════════════╗\n');
fprintf('║      TEST DE INTEGRACIÓN: RRT* + CONTROLADOR                  ║\n');
fprintf('╚════════════════════════════════════════════════════════════════╝\n\n');

%% ════════════════════════════════════════════════════════════════════════
%  GENERAR PATH CON RRT*
%% ════════════════════════════════════════════════════════════════════════
fprintf('► FASE 1: Generando path con RRT*...\n');

map = environment_map('simple');
params = planner_parameters();
params.show_progress = false;
params.max_iterations = 2000;

waypoints = generate_waypoints(map.start, map.goal, map, params);

if isempty(waypoints)
    fprintf('✗ FAIL: No se pudo generar path\n');
    return;
end

fprintf('  ✓ Path generado con %d waypoints\n', size(waypoints, 1));

%% ════════════════════════════════════════════════════════════════════════
%  SIMULAR NAVEGACIÓN
%% ════════════════════════════════════════════════════════════════════════
fprintf('\n► FASE 2: Simulando navegación del robot...\n');

% Cargar parámetros y controlador
robot_params = robot_parameters();
fis = fuzzy_yaw_rate_controller_setup();

% Estado inicial
X = zeros(12, 1);
X(1) = map.start(1);
X(2) = map.start(2);

% Parámetros de simulación
dt = 0.01;
max_time = 180;  % segundos (aumentado para dar tiempo al robot)
waypoint_tolerance = 0.5;  % metros

current_wp_idx = 2;
X_ref_prev = [waypoints(current_wp_idx, :)'; zeros(10, 1)];

% Historial
X_history = [];
t_current = 0;
waypoints_reached = 0;

% Simular
while t_current < max_time
    % Objetivo actual
    current_target = waypoints(current_wp_idx, :)';
    
    % Verificar llegada a waypoint
    dist = norm(X(1:2) - current_target);
    if dist < waypoint_tolerance
        waypoints_reached = waypoints_reached + 1;
        
        if current_wp_idx >= size(waypoints, 1)
            break;  % Meta alcanzada
        end
        current_wp_idx = current_wp_idx + 1;
        current_target = waypoints(current_wp_idx, :)';
    end
    
    % Control
    X_ref = zeros(12, 1);
    X_ref(1:2) = current_target;
    
    torques = fuzzy_yaw_rate_controller(X, X_ref, X_ref_prev, dt, fis, robot_params);
    
    % Integrar
    model_func = @(t, x) skid_steer_robot_model(t, x, torques, robot_params);
    [~, X_ode] = ode45(model_func, [0 dt], X);
    X = X_ode(end, :)';
    
    % Guardar
    X_history = [X_history; X'];
    X_ref_prev = X_ref;
    t_current = t_current + dt;
end

%% ════════════════════════════════════════════════════════════════════════
%  VERIFICAR RESULTADOS
%% ════════════════════════════════════════════════════════════════════════
fprintf('\n► FASE 3: Verificando resultados...\n');

tests_passed = 0;
tests_failed = 0;

% Test 1: ¿Se alcanzó la meta?
final_pos = X(1:2);
dist_to_goal = norm(final_pos - map.goal);
if dist_to_goal < 1.0
    tests_passed = tests_passed + 1;
    fprintf('  ✓ TEST 1 PASSED: Meta alcanzada (dist=%.2fm)\n', dist_to_goal);
else
    tests_failed = tests_failed + 1;
    fprintf('  ✗ TEST 1 FAILED: Meta no alcanzada (dist=%.2fm)\n', dist_to_goal);
end

% Test 2: ¿Se alcanzaron suficientes waypoints?
wp_ratio = waypoints_reached / (size(waypoints, 1) - 1);
if wp_ratio >= 0.8
    tests_passed = tests_passed + 1;
    fprintf('  ✓ TEST 2 PASSED: %.0f%% de waypoints alcanzados\n', wp_ratio * 100);
else
    tests_failed = tests_failed + 1;
    fprintf('  ✗ TEST 2 FAILED: Solo %.0f%% de waypoints alcanzados\n', wp_ratio * 100);
end

% Test 3: ¿El robot evitó obstáculos?
collision_detected = false;
for i = 1:size(X_history, 1)
    pos = X_history(i, 1:2)';
    for j = 1:length(map.obstacles)
        obs = map.obstacles{j};
        if strcmpi(obs.type, 'circle')
            if norm(pos - obs.center) < obs.radius
                collision_detected = true;
                break;
            end
        elseif strcmpi(obs.type, 'rectangle')
            if abs(pos(1) - obs.center(1)) < obs.width/2 && ...
                    abs(pos(2) - obs.center(2)) < obs.height/2
                collision_detected = true;
                break;
            end
        end
    end
    if collision_detected
        break;
    end
end

if ~collision_detected
    tests_passed = tests_passed + 1;
    fprintf('  ✓ TEST 3 PASSED: Sin colisiones detectadas\n');
else
    tests_failed = tests_failed + 1;
    fprintf('  ✗ TEST 3 FAILED: Colisión detectada\n');
end

%% ════════════════════════════════════════════════════════════════════════
%  VISUALIZACIÓN
%% ════════════════════════════════════════════════════════════════════════
figure('Name', 'Test Full Navigation', 'Position', [100 100 900 600]);
hold on; grid on; axis equal;

% Obstáculos
for i = 1:length(map.obstacles)
    obs = map.obstacles{i};
    switch lower(obs.type)
        case 'circle'
            theta = linspace(0, 2*pi, 50);
            x = obs.center(1) + obs.radius * cos(theta);
            y = obs.center(2) + obs.radius * sin(theta);
            fill(x, y, [0.5 0.5 0.5], 'FaceAlpha', 0.5);
        case 'rectangle'
            hw = obs.width/2; hh = obs.height/2;
            x = obs.center(1) + [-hw hw hw -hw];
            y = obs.center(2) + [-hh -hh hh hh];
            fill(x, y, [0.5 0.5 0.5], 'FaceAlpha', 0.5);
    end
end

% Waypoints planificados
plot(waypoints(:,1), waypoints(:,2), 'b--', 'LineWidth', 1.5, 'DisplayName', 'Waypoints RRT*');
plot(waypoints(:,1), waypoints(:,2), 'bo', 'MarkerSize', 8);

% Trayectoria del robot
plot(X_history(:,1), X_history(:,2), 'g-', 'LineWidth', 2, 'DisplayName', 'Trayectoria robot');

% Inicio y meta
plot(map.start(1), map.start(2), 'go', 'MarkerSize', 15, 'LineWidth', 3, 'DisplayName', 'Inicio');
plot(map.goal(1), map.goal(2), 'r*', 'MarkerSize', 15, 'LineWidth', 3, 'DisplayName', 'Meta');
plot(final_pos(1), final_pos(2), 'rx', 'MarkerSize', 15, 'LineWidth', 3, 'DisplayName', 'Posición final');

xlim([map.x_min map.x_max]);
ylim([map.y_min map.y_max]);
xlabel('X (m)'); ylabel('Y (m)');
title(sprintf('Test de Navegación | Passed: %d, Failed: %d', tests_passed, tests_failed));
legend('Location', 'best');

%% ════════════════════════════════════════════════════════════════════════
%  RESUMEN
%% ════════════════════════════════════════════════════════════════════════
fprintf('\n╔════════════════════════════════════════════════════════════════╗\n');
fprintf('║                   RESUMEN DE TESTS                            ║\n');
fprintf('╠════════════════════════════════════════════════════════════════╣\n');
fprintf('║  Tests pasados:  %d                                           ║\n', tests_passed);
fprintf('║  Tests fallidos: %d                                           ║\n', tests_failed);
fprintf('║  Tiempo simulación: %.2f s                                   ║\n', t_current);
fprintf('╚════════════════════════════════════════════════════════════════╝\n\n');

if tests_failed == 0
    fprintf('✓✓✓ INTEGRACIÓN EXITOSA ✓✓✓\n\n');
else
    fprintf('✗✗✗ HAY PROBLEMAS EN LA INTEGRACIÓN ✗✗✗\n\n');
end

end
