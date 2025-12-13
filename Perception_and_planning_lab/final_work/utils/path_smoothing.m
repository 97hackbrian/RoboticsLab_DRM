function smoothed_path = path_smoothing(path, obstacles, margin, params)
% path_smoothing: Suaviza y optimiza el path generado por RRT*
%
% ENTRADAS:
%   path - Path original [Nx2] array de puntos
%   obstacles - Cell array de obstáculos
%   margin - Margen de seguridad (radio robot)
%   params - Parámetros del planificador
%
% SALIDA:
%   smoothed_path - Path suavizado [Mx2] array de waypoints

if nargin < 4
    params = struct();
    params.smoothing_iterations = 100;
    params.min_waypoint_spacing = 0.3;
    params.max_waypoint_spacing = 1.5;
end

if size(path, 1) <= 2
    smoothed_path = path;
    return;
end

%% Paso 1: Shortcut Smoothing
% Intenta conectar puntos no adyacentes para acortar el path
current_path = path;

for iter = 1:params.smoothing_iterations
    n = size(current_path, 1);
    if n <= 2
        break;
    end
    
    % Seleccionar dos índices aleatorios
    i = randi(n-1);
    j = i + randi(n - i);
    
    if j - i <= 1
        continue;  % Ya son adyacentes
    end
    
    p1 = current_path(i, :)';
    p2 = current_path(j, :)';
    
    % Verificar si la conexión directa es libre de colisiones
    if ~collision_check(p1, p2, obstacles, margin)
        % Eliminar puntos intermedios
        current_path = [current_path(1:i, :); current_path(j:end, :)];
    end
end

%% Paso 2: Interpolación para asegurar espaciado uniforme
smoothed_path = interpolate_path(current_path, params.min_waypoint_spacing, ...
    params.max_waypoint_spacing);

%% Paso 3: Verificación final de colisiones
% Asegurar que el path suavizado sigue siendo válido
for i = 1:(size(smoothed_path, 1) - 1)
    p1 = smoothed_path(i, :)';
    p2 = smoothed_path(i+1, :)';
    
    if collision_check(p1, p2, obstacles, margin)
        warning('Path suavizado tiene colisión. Usando path sin suavizar.');
        smoothed_path = path;
        return;
    end
end

fprintf('► Path suavizado: %d puntos → %d waypoints\n', size(path, 1), size(smoothed_path, 1));

end

%% ════════════════════════════════════════════════════════════════════════
%  FUNCIONES AUXILIARES
%% ════════════════════════════════════════════════════════════════════════

function interp_path = interpolate_path(path, min_spacing, max_spacing)
% Interpola el path para tener waypoints con espaciado uniforme

interp_path = path(1, :);  % Empezar con el primer punto

for i = 1:(size(path, 1) - 1)
    p1 = path(i, :);
    p2 = path(i+1, :);
    
    seg_length = norm(p2 - p1);
    
    if seg_length <= max_spacing
        % Segmento corto, solo agregar el punto final
        interp_path = [interp_path; p2];
    else
        % Segmento largo, subdividir
        num_points = ceil(seg_length / max_spacing);
        
        for j = 1:num_points
            t = j / num_points;
            p_interp = p1 + t * (p2 - p1);
            interp_path = [interp_path; p_interp];
        end
    end
end

% Eliminar puntos duplicados o muy cercanos
cleaned_path = interp_path(1, :);
for i = 2:size(interp_path, 1)
    if norm(interp_path(i, :) - cleaned_path(end, :)) >= min_spacing
        cleaned_path = [cleaned_path; interp_path(i, :)];
    end
end

% Asegurar que el último punto siempre esté incluido
if norm(cleaned_path(end, :) - path(end, :)) > 1e-6
    cleaned_path = [cleaned_path; path(end, :)];
end

interp_path = cleaned_path;
end
