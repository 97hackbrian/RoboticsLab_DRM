function waypoints = generate_waypoints(start, goal, map, params)
% generate_waypoints: Genera waypoints usando RRT* para navegación del robot
%
% ENTRADAS:
%   start - Posición inicial [x; y]
%   goal - Posición objetivo [x; y]
%   map - Estructura del mapa (de environment_map.m)
%   params - Parámetros del planificador (de planner_parameters.m)
%
% SALIDA:
%   waypoints - [Nx2] array de waypoints suavizados para el controlador
%               Cada fila es [x, y] de un waypoint

fprintf('\n╔════════════════════════════════════════════════════════════╗\n');
fprintf('║         GENERADOR DE WAYPOINTS CON RRT*                   ║\n');
fprintf('╚════════════════════════════════════════════════════════════╝\n');

%% 1. Ejecutar RRT* para obtener path crudo
[path_raw, tree, stats] = rrt_star(start, goal, map, params);

if isempty(path_raw)
    warning('No se pudo generar path. Devolviendo waypoints vacíos.');
    waypoints = [];
    return;
end

%% 2. Suavizar path
fprintf('► Suavizando path...\n');
path_smooth = path_smoothing(path_raw, map.obstacles, map.robot_radius, params);

%% 3. Convertir a formato de waypoints
waypoints = path_smooth;

%% 4. Estadísticas
fprintf('\n════════════════════════════════════════════════════════════\n');
fprintf('             RESUMEN DE GENERACIÓN DE WAYPOINTS             \n');
fprintf('════════════════════════════════════════════════════════════\n');
fprintf('  Nodos explorados:      %d\n', stats.nodes_explored);
fprintf('  Iteraciones:           %d\n', stats.iterations);
fprintf('  Tiempo planificación:  %.2f s\n', stats.planning_time);
fprintf('  Path crudo:            %d puntos\n', size(path_raw, 1));
fprintf('  Waypoints finales:     %d puntos\n', size(waypoints, 1));
fprintf('  Longitud total path:   %.2f m\n', compute_total_length(waypoints));
fprintf('════════════════════════════════════════════════════════════\n\n');

%% 5. Visualización de comparación
figure('Name', 'Waypoints Generados', 'Position', [150 150 800 600]);
ax = gca;
hold on; grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)');
title('Comparación: Path Crudo vs Waypoints Suavizados');

% Límites del mapa
xlim([map.x_min map.x_max]);
ylim([map.y_min map.y_max]);

% Dibujar obstáculos
draw_obstacles_local(ax, map.obstacles);

% Path crudo
plot(path_raw(:,1), path_raw(:,2), 'b--', 'LineWidth', 1.5, ...
    'DisplayName', sprintf('Path RRT* (%d pts)', size(path_raw,1)));

% Waypoints suavizados
plot(waypoints(:,1), waypoints(:,2), 'r-', 'LineWidth', 2.5, ...
    'DisplayName', sprintf('Waypoints (%d pts)', size(waypoints,1)));
plot(waypoints(:,1), waypoints(:,2), 'ro', 'MarkerSize', 10, ...
    'MarkerFaceColor', 'r', 'HandleVisibility', 'off');

% Numerar waypoints
for i = 1:size(waypoints, 1)
    text(waypoints(i,1) + 0.3, waypoints(i,2) + 0.3, num2str(i), ...
        'FontSize', 9, 'FontWeight', 'bold', 'Color', [0.6 0 0]);
end

% Inicio y meta
plot(start(1), start(2), 'go', 'MarkerSize', 18, 'LineWidth', 3, ...
    'DisplayName', 'Inicio');
plot(goal(1), goal(2), 'r*', 'MarkerSize', 18, 'LineWidth', 3, ...
    'DisplayName', 'Meta');

legend('Location', 'best');

end

%% ════════════════════════════════════════════════════════════════════════
%  FUNCIONES AUXILIARES
%% ════════════════════════════════════════════════════════════════════════

function len = compute_total_length(path)
len = 0;
for i = 1:(size(path,1)-1)
    len = len + norm(path(i+1,:) - path(i,:));
end
end

function draw_obstacles_local(ax, obstacles)
for i = 1:length(obstacles)
    obs = obstacles{i};
    switch lower(obs.type)
        case 'circle'
            theta = linspace(0, 2*pi, 50);
            x = obs.center(1) + obs.radius * cos(theta);
            y = obs.center(2) + obs.radius * sin(theta);
            fill(ax, x, y, [0.4 0.4 0.4], 'FaceAlpha', 0.5, ...
                'EdgeColor', 'k', 'LineWidth', 1.5, 'HandleVisibility', 'off');
        case 'rectangle'
            hw = obs.width/2; hh = obs.height/2;
            x = obs.center(1) + [-hw hw hw -hw];
            y = obs.center(2) + [-hh -hh hh hh];
            fill(ax, x, y, [0.4 0.4 0.4], 'FaceAlpha', 0.5, ...
                'EdgeColor', 'k', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    end
end
end
