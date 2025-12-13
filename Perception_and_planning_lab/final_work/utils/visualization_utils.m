function visualization_utils()
% visualization_utils: Funciones de visualización para RRT* y navegación
% Este archivo contiene múltiples funciones que deben llamarse directamente
%
% Funciones disponibles:
%   draw_obstacles(ax, obstacles)
%   draw_tree(ax, tree)
%   draw_path(ax, path, color, linewidth)
%   draw_waypoints(ax, waypoints, color)
%   draw_robot_2d(ax, x, y, theta, robot_radius)

% Este archivo es un contenedor de documentación
% Las funciones están definidas abajo como funciones separadas

fprintf('visualization_utils: Usar funciones individuales:\n');
fprintf('  - draw_obstacles(ax, obstacles)\n');
fprintf('  - draw_tree(ax, tree)\n');
fprintf('  - draw_path(ax, path, color, linewidth)\n');
fprintf('  - draw_waypoints(ax, waypoints, color)\n');
fprintf('  - draw_robot_2d(ax, x, y, theta, robot_radius)\n');

end

%% ════════════════════════════════════════════════════════════════════════
%  FUNCIONES DE VISUALIZACIÓN
%% ════════════════════════════════════════════════════════════════════════

function h = draw_obstacles(ax, obstacles, color)
% Dibuja todos los obstáculos en el eje especificado
%
% ENTRADAS:
%   ax - Handle del eje
%   obstacles - Cell array de obstáculos
%   color - (opcional) Color de los obstáculos

if nargin < 3
    color = [0.3 0.3 0.3];  % Gris oscuro por defecto
end

h = [];

for i = 1:length(obstacles)
    obs = obstacles{i};
    
    switch lower(obs.type)
        case 'circle'
            theta = linspace(0, 2*pi, 50);
            x_circle = obs.center(1) + obs.radius * cos(theta);
            y_circle = obs.center(2) + obs.radius * sin(theta);
            h_obs = fill(ax, x_circle, y_circle, color, ...
                'FaceAlpha', 0.6, 'EdgeColor', 'k', 'LineWidth', 1.5);
            
        case 'rectangle'
            half_w = obs.width / 2;
            half_h = obs.height / 2;
            x_rect = obs.center(1) + [-half_w, half_w, half_w, -half_w];
            y_rect = obs.center(2) + [-half_h, -half_h, half_h, half_h];
            h_obs = fill(ax, x_rect, y_rect, color, ...
                'FaceAlpha', 0.6, 'EdgeColor', 'k', 'LineWidth', 1.5);
    end
    
    h = [h; h_obs];
end

end

function h = draw_tree(ax, tree, color)
% Dibuja el árbol RRT* (nodos y aristas)
%
% ENTRADAS:
%   ax - Handle del eje
%   tree - Estructura del árbol con .nodes y .parents
%   color - (opcional) Color del árbol

if nargin < 3
    color = [0.7 0.7 0.7];  % Gris claro
end

h = [];

% Dibujar aristas
for i = 2:size(tree.nodes, 1)
    parent_idx = tree.parents(i);
    if parent_idx > 0
        x_line = [tree.nodes(parent_idx, 1), tree.nodes(i, 1)];
        y_line = [tree.nodes(parent_idx, 2), tree.nodes(i, 2)];
        h_edge = plot(ax, x_line, y_line, '-', 'Color', color, 'LineWidth', 0.5);
        h = [h; h_edge];
    end
end

% Dibujar nodos
h_nodes = plot(ax, tree.nodes(:,1), tree.nodes(:,2), '.', ...
    'Color', color * 0.7, 'MarkerSize', 3);
h = [h; h_nodes];

end

function h = draw_path(ax, path, color, linewidth)
% Dibuja el path encontrado
%
% ENTRADAS:
%   ax - Handle del eje
%   path - [Nx2] array de puntos del path
%   color - (opcional) Color del path
%   linewidth - (opcional) Grosor de línea

if nargin < 3 || isempty(color)
    color = [0.2 0.6 0.2];  % Verde
end
if nargin < 4
    linewidth = 2.5;
end

h = plot(ax, path(:,1), path(:,2), '-', 'Color', color, ...
    'LineWidth', linewidth);

end

function h = draw_waypoints(ax, waypoints, color)
% Dibuja los waypoints como marcadores
%
% ENTRADAS:
%   ax - Handle del eje
%   waypoints - [Nx2] array de waypoints
%   color - (opcional) Color de los marcadores

if nargin < 3
    color = [0.8 0.2 0.2];  % Rojo
end

h = [];

% Dibujar círculos en cada waypoint
for i = 1:size(waypoints, 1)
    h_wp = plot(ax, waypoints(i,1), waypoints(i,2), 'o', ...
        'Color', color, 'MarkerFaceColor', color, ...
        'MarkerSize', 8);
    h = [h; h_wp];
    
    % Número del waypoint
    text(ax, waypoints(i,1) + 0.3, waypoints(i,2) + 0.3, ...
        num2str(i), 'FontSize', 8, 'Color', color * 0.7);
end

end

function h = draw_robot_2d(ax, x, y, theta, robot_radius, color)
% Dibuja el robot como un círculo con flecha de orientación
%
% ENTRADAS:
%   ax - Handle del eje
%   x, y - Posición del robot
%   theta - Orientación (rad)
%   robot_radius - Radio del robot
%   color - (opcional) Color del robot

if nargin < 6
    color = [0 0.5 1];  % Azul
end

% Cuerpo (círculo)
theta_circle = linspace(0, 2*pi, 30);
x_circle = x + robot_radius * cos(theta_circle);
y_circle = y + robot_radius * sin(theta_circle);
h_body = fill(ax, x_circle, y_circle, color, ...
    'FaceAlpha', 0.5, 'EdgeColor', color * 0.7, 'LineWidth', 2);

% Flecha de orientación
arrow_len = robot_radius * 1.5;
h_arrow = quiver(ax, x, y, arrow_len*cos(theta), arrow_len*sin(theta), 0, ...
    'Color', 'r', 'LineWidth', 2, 'MaxHeadSize', 0.8);

h = [h_body; h_arrow];

end

function setup_plot_environment(ax, map)
% Configura el entorno de visualización
%
% ENTRADAS:
%   ax - Handle del eje
%   map - Estructura del mapa

hold(ax, 'on');
grid(ax, 'on');
axis(ax, 'equal');
xlabel(ax, 'X (m)');
ylabel(ax, 'Y (m)');
xlim(ax, [map.x_min map.x_max]);
ylim(ax, [map.y_min map.y_max]);

% Dibujar obstáculos
draw_obstacles(ax, map.obstacles);

% Dibujar inicio y meta
plot(ax, map.start(1), map.start(2), 'go', 'MarkerSize', 15, ...
    'LineWidth', 3, 'MarkerFaceColor', 'g');
plot(ax, map.goal(1), map.goal(2), 'r*', 'MarkerSize', 15, ...
    'LineWidth', 3);

% Etiquetas
text(ax, map.start(1), map.start(2) - 1, 'START', ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'Color', 'g');
text(ax, map.goal(1), map.goal(2) + 1, 'GOAL', ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'Color', 'r');

end
