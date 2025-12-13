function params = planner_parameters()
% planner_parameters: Parámetros del algoritmo RRT*
%
% SALIDA:
%   params - Estructura con parámetros de configuración del planificador

%% Parámetros del Algoritmo RRT*

% Número máximo de iteraciones del árbol
params.max_iterations = 3000;

% Tamaño máximo de paso para extender el árbol (metros)
% Valores pequeños = paths más suaves pero más iteraciones necesarias
params.step_size = 0.8;

% Probabilidad de muestrear directamente el objetivo (goal bias)
% Valores altos = convergencia más rápida pero puede quedar atrapado
params.goal_bias = 0.10;  % 10% de probabilidad

% Radio de búsqueda para vecinos (rewiring)
% Se calcula dinámicamente como: min(gamma * (log(n)/n)^(1/d), step_size)
% donde gamma depende del espacio libre
params.neighbor_radius_factor = 50.0;  % Factor gamma

% Distancia para considerar que se alcanzó el objetivo
params.goal_tolerance = 0.5;  % metros

% Número mínimo de iteraciones después de encontrar el objetivo
% Permite optimizar el path encontrado
params.min_iterations_after_goal = 500;

%% Parámetros de Suavizado de Path

% Número de intentos de shortcut smoothing
params.smoothing_iterations = 100;

% Resolución de interpolación para verificar colisiones (metros)
params.collision_check_resolution = 0.1;

% Distancia mínima entre waypoints finales
params.min_waypoint_spacing = 0.3;

% Distancia máxima entre waypoints finales
params.max_waypoint_spacing = 1.5;

%% Parámetros de Visualización

% Mostrar progreso durante planificación
params.show_progress = true;

% Frecuencia de actualización visual (cada N iteraciones)
params.plot_frequency = 100;

% Colores para visualización
params.color_tree = [0.7 0.7 0.7];      % Gris para el árbol
params.color_path = [0.2 0.6 0.2];      % Verde para el path
params.color_smooth_path = [0.8 0.2 0.2]; % Rojo para path suavizado
params.color_start = [0 0.5 1];          % Azul para inicio
params.color_goal = [1 0 0];             % Rojo para meta

%% Parámetros de Control (para integración con el robot)

% Velocidad deseada del robot durante navegación
params.desired_velocity = 0.8;  % m/s

% Tiempo de espera en cada waypoint (segundos)
params.waypoint_dwell_time = 0.0;

% Tolerancia para considerar waypoint alcanzado
params.waypoint_tolerance = 0.3;  % metros

fprintf('► Parámetros RRT* cargados:\n');
fprintf('  max_iter=%d, step=%.2fm, goal_bias=%.0f%%, goal_tol=%.2fm\n', ...
    params.max_iterations, params.step_size, params.goal_bias*100, params.goal_tolerance);

end
