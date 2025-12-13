% demo_rrt_star_planner.m
% Demostración del algoritmo RRT* sin navegación del robot
% Visualiza el crecimiento del árbol y la optimización del path

function demo_rrt_star_planner()
close all; clc;

%% Agregar paths
addpath('../config');
addpath('../planners');
addpath('../utils');

fprintf('\n╔════════════════════════════════════════════════════════════════╗\n');
fprintf('║          DEMOSTRACIÓN DEL ALGORITMO RRT*                      ║\n');
fprintf('║   Visualización del árbol de exploración y path óptimo        ║\n');
fprintf('╚════════════════════════════════════════════════════════════════╝\n\n');

%% Seleccionar escenario
fprintf('Escenarios disponibles:\n');
fprintf('  1. empty     - Entorno vacío\n');
fprintf('  2. simple    - Obstáculos simples\n');
fprintf('  3. maze      - Laberinto\n');
fprintf('  4. narrow    - Pasajes estrechos\n');
fprintf('  5. random    - Obstáculos aleatorios\n');
fprintf('  6. challenge - Escenario desafiante\n\n');

scenario = input('Seleccione escenario (1-6) [2]: ');
if isempty(scenario)
    scenario = 2;
end

scenarios = {'empty', 'simple', 'maze', 'narrow', 'random', 'challenge'};
scenario_name = scenarios{min(max(scenario, 1), 6)};

fprintf('\n► Ejecutando escenario: %s\n', scenario_name);

%% Cargar configuraciones
map = environment_map(scenario_name);
params = planner_parameters();

% Configurar para visualización
params.show_progress = true;
params.plot_frequency = 50;

%% Ejecutar RRT*
fprintf('\n► Iniciando planificación RRT*...\n');
tic;

[path, tree, stats] = rrt_star(map.start, map.goal, map, params);

planning_time = toc;

%% Mostrar resultados
if ~isempty(path)
    fprintf('\n════════════════════════════════════════════════════════════════\n');
    fprintf('                   RESULTADOS DE PLANIFICACIÓN                   \n');
    fprintf('════════════════════════════════════════════════════════════════\n');
    fprintf('  ✓ Path encontrado\n');
    fprintf('  Nodos en el árbol:     %d\n', size(tree.nodes, 1));
    fprintf('  Puntos en path:        %d\n', size(path, 1));
    fprintf('  Longitud del path:     %.2f m\n', stats.path_length);
    fprintf('  Tiempo de planning:    %.2f s\n', planning_time);
    fprintf('  Meta encontrada en:    iteración %d\n', stats.goal_found_iter);
    fprintf('════════════════════════════════════════════════════════════════\n');
    
    % Suavizar el path
    fprintf('\n► Aplicando suavizado al path...\n');
    smooth_path = path_smoothing(path, map.obstacles, map.robot_radius, params);
    
    % Mostrar comparación
    figure('Name', 'Comparación de Paths', 'Position', [200 200 900 600]);
    hold on; grid on; axis equal;
    
    % Dibujar obstáculos
    for i = 1:length(map.obstacles)
        obs = map.obstacles{i};
        switch lower(obs.type)
            case 'circle'
                theta = linspace(0, 2*pi, 50);
                x = obs.center(1) + obs.radius * cos(theta);
                y = obs.center(2) + obs.radius * sin(theta);
                fill(x, y, [0.5 0.5 0.5], 'FaceAlpha', 0.5, 'EdgeColor', 'k');
            case 'rectangle'
                hw = obs.width/2; hh = obs.height/2;
                x = obs.center(1) + [-hw hw hw -hw];
                y = obs.center(2) + [-hh -hh hh hh];
                fill(x, y, [0.5 0.5 0.5], 'FaceAlpha', 0.5, 'EdgeColor', 'k');
        end
    end
    
    xlim([map.x_min map.x_max]);
    ylim([map.y_min map.y_max]);
    
    % Dibujar path original y suavizado
    plot(path(:,1), path(:,2), 'b--', 'LineWidth', 1.5, ...
        'DisplayName', sprintf('RRT* original (%d pts, %.2fm)', size(path,1), stats.path_length));
    plot(smooth_path(:,1), smooth_path(:,2), 'r-', 'LineWidth', 2.5, ...
        'DisplayName', sprintf('Path suavizado (%d pts)', size(smooth_path,1)));
    
    % Marcar waypoints
    plot(smooth_path(:,1), smooth_path(:,2), 'ro', 'MarkerSize', 8, ...
        'MarkerFaceColor', 'r', 'HandleVisibility', 'off');
    
    % Marcar inicio y meta
    plot(map.start(1), map.start(2), 'go', 'MarkerSize', 18, 'LineWidth', 3, ...
        'DisplayName', 'Inicio');
    plot(map.goal(1), map.goal(2), 'r*', 'MarkerSize', 18, 'LineWidth', 3, ...
        'DisplayName', 'Meta');
    
    title(sprintf('RRT* Path Planning - Escenario: %s', scenario_name));
    xlabel('X (m)'); ylabel('Y (m)');
    legend('Location', 'best');
    
else
    fprintf('\n✗ No se encontró un path válido.\n');
    fprintf('  Intente con otro escenario o aumente max_iterations.\n');
end

fprintf('\n► Demo completada.\n\n');

end
