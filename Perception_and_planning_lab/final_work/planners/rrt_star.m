function [path, tree, stats] = rrt_star(start, goal, map, params)
% rrt_star: Algoritmo RRT* (Rapidly-exploring Random Tree Star)
% Genera un path óptimo desde start hasta goal evitando obstáculos
%
% ENTRADAS:
%   start - Posición inicial [x; y]
%   goal - Posición objetivo [x; y]
%   map - Estructura del mapa (de environment_map.m)
%   params - Parámetros del planificador (de planner_parameters.m)
%
% SALIDAS:
%   path - [Nx2] array con el path encontrado (vacío si no se encontró)
%   tree - Estructura del árbol con nodes, parents, costs
%   stats - Estadísticas de planificación

fprintf('\n╔════════════════════════════════════════════════════════════╗\n');
fprintf('║           RRT* PATH PLANNING ALGORITHM                    ║\n');
fprintf('╚════════════════════════════════════════════════════════════╝\n');

%% 1. INICIALIZACIÓN
tic;

% Inicializar árbol con nodo inicial
tree.nodes = start(:)';       % [Nx2] matriz de nodos
tree.parents = 0;             % Índice del padre (0 = raíz)
tree.costs = 0;               % Costo desde el inicio

% Variables de control
goal_reached = false;
goal_node_idx = -1;
best_cost = inf;

% Estadísticas
stats.iterations = 0;
stats.nodes_explored = 1;
stats.goal_found_iter = -1;
stats.planning_time = 0;
stats.path_length = 0;
stats.path_cost = 0;

% Dimensiones del espacio
x_range = [map.x_min, map.x_max];
y_range = [map.y_min, map.y_max];
margin = map.robot_radius;
obstacles = map.obstacles;

% Visualización si está habilitada
if params.show_progress
    fig = figure('Name', 'RRT* Planning', 'Position', [100 100 800 700]);
    ax = axes(fig);
    setup_visualization(ax, map);
    drawnow;
end

fprintf('► Inicio: (%.2f, %.2f) → Meta: (%.2f, %.2f)\n', ...
    start(1), start(2), goal(1), goal(2));
fprintf('► Parámetros: max_iter=%d, step=%.2f, goal_bias=%.2f\n', ...
    params.max_iterations, params.step_size, params.goal_bias);

%% 2. BUCLE PRINCIPAL RRT*
iterations_after_goal = 0;

for iter = 1:params.max_iterations
    stats.iterations = iter;
    
    %% 2.1 Muestrear punto aleatorio (con goal bias)
    if rand() < params.goal_bias && ~goal_reached
        q_rand = goal(:)';
    else
        q_rand = [x_range(1) + rand()*(x_range(2)-x_range(1)), ...
            y_range(1) + rand()*(y_range(2)-y_range(1))];
    end
    
    %% 2.2 Encontrar nodo más cercano
    [nearest_idx, ~] = find_nearest(tree.nodes, q_rand);
    q_nearest = tree.nodes(nearest_idx, :);
    
    %% 2.3 Extender hacia el nuevo punto
    q_new = steer(q_nearest, q_rand, params.step_size);
    
    %% 2.4 Verificar colisión
    if collision_check(q_nearest', q_new', obstacles, margin)
        continue;  % Saltar si hay colisión
    end
    
    %% 2.5 Encontrar vecinos dentro del radio
    n = size(tree.nodes, 1);
    radius = min(params.neighbor_radius_factor * sqrt(log(n+1)/(n+1)), ...
        params.step_size * 2);
    neighbor_idxs = find_neighbors(tree.nodes, q_new, radius);
    
    %% 2.6 Elegir mejor padre (minimizar costo)
    best_parent = nearest_idx;
    best_cost_to_new = tree.costs(nearest_idx) + norm(q_new - q_nearest);
    
    for k = 1:length(neighbor_idxs)
        idx = neighbor_idxs(k);
        q_neighbor = tree.nodes(idx, :);
        
        if ~collision_check(q_neighbor', q_new', obstacles, margin)
            cost_via_neighbor = tree.costs(idx) + norm(q_new - q_neighbor);
            if cost_via_neighbor < best_cost_to_new
                best_parent = idx;
                best_cost_to_new = cost_via_neighbor;
            end
        end
    end
    
    %% 2.7 Agregar nuevo nodo al árbol
    tree.nodes = [tree.nodes; q_new];
    tree.parents = [tree.parents; best_parent];
    tree.costs = [tree.costs; best_cost_to_new];
    new_node_idx = size(tree.nodes, 1);
    stats.nodes_explored = new_node_idx;
    
    %% 2.8 Rewiring (optimizar vecinos existentes)
    for k = 1:length(neighbor_idxs)
        idx = neighbor_idxs(k);
        q_neighbor = tree.nodes(idx, :);
        
        cost_via_new = best_cost_to_new + norm(q_neighbor - q_new);
        
        if cost_via_new < tree.costs(idx)
            if ~collision_check(q_new', q_neighbor', obstacles, margin)
                tree.parents(idx) = new_node_idx;
                tree.costs(idx) = cost_via_new;
                
                % Propagar cambio de costo a descendientes
                tree = propagate_cost_update(tree, idx);
            end
        end
    end
    
    %% 2.9 Verificar si llegamos al objetivo
    dist_to_goal = norm(q_new - goal(:)');
    if dist_to_goal < params.goal_tolerance
        if ~goal_reached
            goal_reached = true;
            stats.goal_found_iter = iter;
            fprintf('► ¡META ALCANZADA en iteración %d! Optimizando...\n', iter);
        end
        
        % Actualizar mejor camino si este es mejor
        if best_cost_to_new + dist_to_goal < best_cost
            goal_node_idx = new_node_idx;
            best_cost = best_cost_to_new + dist_to_goal;
        end
    end
    
    %% 2.10 Condición de parada después de encontrar objetivo
    if goal_reached
        iterations_after_goal = iterations_after_goal + 1;
        if iterations_after_goal >= params.min_iterations_after_goal
            fprintf('► Optimización completa tras %d iteraciones adicionales.\n', ...
                iterations_after_goal);
            break;
        end
    end
    
    %% 2.11 Visualización periódica
    if params.show_progress && mod(iter, params.plot_frequency) == 0
        update_visualization(ax, tree, goal_reached, goal_node_idx, map);
        title(ax, sprintf('RRT* Iteración %d | Nodos: %d | Meta: %s', ...
            iter, size(tree.nodes,1), string(goal_reached)));
        drawnow limitrate;
    end
end

%% 3. EXTRAER PATH
if goal_node_idx > 0
    path = extract_path(tree, goal_node_idx);
    path = [path; goal(:)'];  % Agregar meta exacta
    stats.path_length = compute_path_length(path);
    stats.path_cost = best_cost;
    fprintf('✓ Path encontrado con %d nodos, longitud: %.2f m\n', ...
        size(path,1), stats.path_length);
else
    path = [];
    fprintf('✗ No se encontró path al objetivo.\n');
end

stats.planning_time = toc;
fprintf('► Tiempo de planificación: %.2f segundos\n\n', stats.planning_time);

%% 4. VISUALIZACIÓN FINAL
if params.show_progress && ~isempty(path)
    plot(ax, path(:,1), path(:,2), 'g-', 'LineWidth', 3);
    plot(ax, path(:,1), path(:,2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
    title(ax, sprintf('RRT* Completo | Path: %.2fm | Tiempo: %.2fs', ...
        stats.path_length, stats.planning_time));
    drawnow;
end

end

%% ════════════════════════════════════════════════════════════════════════
%  FUNCIONES AUXILIARES
%% ════════════════════════════════════════════════════════════════════════

function [idx, dist] = find_nearest(nodes, q)
% Encuentra el nodo más cercano a q
dists = vecnorm(nodes - q, 2, 2);
[dist, idx] = min(dists);
end

function idxs = find_neighbors(nodes, q, radius)
% Encuentra todos los nodos dentro del radio
dists = vecnorm(nodes - q, 2, 2);
idxs = find(dists <= radius);
end

function q_new = steer(q_from, q_to, step_size)
% Mueve desde q_from hacia q_to con paso máximo step_size
direction = q_to - q_from;
dist = norm(direction);
if dist <= step_size
    q_new = q_to;
else
    q_new = q_from + (direction / dist) * step_size;
end
end

function tree = propagate_cost_update(tree, idx)
% Propaga actualización de costo a todos los descendientes
children = find(tree.parents == idx);
for c = children'
    parent_cost = tree.costs(tree.parents(c));
    edge_cost = norm(tree.nodes(c,:) - tree.nodes(tree.parents(c),:));
    tree.costs(c) = parent_cost + edge_cost;
    tree = propagate_cost_update(tree, c);
end
end

function path = extract_path(tree, goal_idx)
% Extrae el path desde la raíz hasta goal_idx
path = tree.nodes(goal_idx, :);
current = goal_idx;
while tree.parents(current) > 0
    current = tree.parents(current);
    path = [tree.nodes(current, :); path];
end
end

function len = compute_path_length(path)
% Calcula la longitud total del path
len = 0;
for i = 1:(size(path,1)-1)
    len = len + norm(path(i+1,:) - path(i,:));
end
end

function setup_visualization(ax, map)
% Configura la visualización inicial
hold(ax, 'on');
grid(ax, 'on');
axis(ax, 'equal');
xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)');
xlim(ax, [map.x_min map.x_max]);
ylim(ax, [map.y_min map.y_max]);

% Dibujar obstáculos
for i = 1:length(map.obstacles)
    obs = map.obstacles{i};
    switch lower(obs.type)
        case 'circle'
            theta = linspace(0, 2*pi, 50);
            x = obs.center(1) + obs.radius * cos(theta);
            y = obs.center(2) + obs.radius * sin(theta);
            fill(ax, x, y, [0.3 0.3 0.3], 'FaceAlpha', 0.6, 'EdgeColor', 'k');
        case 'rectangle'
            hw = obs.width/2; hh = obs.height/2;
            x = obs.center(1) + [-hw hw hw -hw];
            y = obs.center(2) + [-hh -hh hh hh];
            fill(ax, x, y, [0.3 0.3 0.3], 'FaceAlpha', 0.6, 'EdgeColor', 'k');
    end
end

% Inicio y meta
plot(ax, map.start(1), map.start(2), 'go', 'MarkerSize', 15, 'LineWidth', 3);
plot(ax, map.goal(1), map.goal(2), 'r*', 'MarkerSize', 15, 'LineWidth', 3);
end

function update_visualization(ax, tree, goal_reached, goal_idx, map)
% Actualiza la visualización del árbol
persistent h_tree

% Limpiar árbol anterior
if ~isempty(h_tree) && all(isvalid(h_tree))
    delete(h_tree);
end
h_tree = [];

% Dibujar aristas del árbol
for i = 2:size(tree.nodes, 1)
    p = tree.parents(i);
    if p > 0
        h = plot(ax, [tree.nodes(p,1) tree.nodes(i,1)], ...
            [tree.nodes(p,2) tree.nodes(i,2)], ...
            'Color', [0.7 0.7 0.7 0.5], 'LineWidth', 0.5);
        h_tree = [h_tree; h];
    end
end

% Si hay path, dibujarlo
if goal_reached && goal_idx > 0
    path = extract_path(tree, goal_idx);
    h = plot(ax, path(:,1), path(:,2), 'b-', 'LineWidth', 2);
    h_tree = [h_tree; h];
end
end
