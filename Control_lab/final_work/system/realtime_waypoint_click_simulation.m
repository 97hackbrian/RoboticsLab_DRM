% realtime_waypoint_click_simulation.m
% Simulación interactiva en tiempo real donde el usuario define waypoints con clicks

function realtime_waypoint_click_simulation()
close all;

% Agregar rutas
addpath('../config');
addpath('../system');
addpath('../controllers');

fprintf('\n╔════════════════════════════════════════════════════════════╗\n');
fprintf('║   SIMULACIÓN INTERACTIVA - CLICK TO DRIVE                 ║\n');
fprintf('╚════════════════════════════════════════════════════════════╝\n');
fprintf('► Instrucciones:\n');
fprintf('   1. Haz CLICK en el plano XY para establecer un nuevo objetivo.\n');
fprintf('   2. El robot navegará automáticamente hacia ese punto.\n');
fprintf('   3. Cierra la ventana para terminar.\n\n');

%% 1. CONFIGURACIÓN
params = robot_parameters();
fis = fuzzy_yaw_rate_controller_setup();

% Estado Inicial
X = zeros(12, 1);
X(6) = 0; % Yaw inicial 0

% Objetivo inicial (delante del robot)
current_target = [5; 0];

% Variables de control de tiempo
dt = 0.05; % Paso de simulación (más lento que 0.01 para "tiempo real" visual)
time_multiplier = 1.0; % Factor de velocidad (1.0 = tiempo real aprox)

%% 2. PREPARAR VISUALIZACIÓN (MODO 2D)
fig = figure('Name', 'Interactive Robot Control (2D)', 'NumberTitle', 'off', ...
    'Position', [100 100 1000 800], 'Color', 'w');

ax = axes(fig);
hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
xlabel('X (m)'); ylabel('Y (m)');

% RANGOS MÁXIMOS DEL CAMPO DE PRUEBAS
field_xlim = [-15 25];
field_ylim = [-15 25];
xlim(ax, field_xlim); ylim(ax, field_ylim);

% Dibujar Robot (2D)
% Usamos un objeto 'transform' o simplemente actualizamos los datos
% Cuerpo (Rectángulo)
robot_body = patch('XData', [], 'YData', [], 'FaceColor', 'b', 'FaceAlpha', 0.3, 'EdgeColor', 'k', 'LineWidth', 2);
% Dirección (Flecha)
heading_arrow = quiver(0,0, 1,0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'AutoScale', 'off');

% Trayectorias (Estela)
trail_plot = plot(0,0, 'b:', 'LineWidth', 1);
X_trail = []; Y_trail = [];

% Dibujar Objetivo
target_marker = plot(current_target(1), current_target(2), 'rx', 'MarkerSize', 15, 'LineWidth', 3);
target_circle = plot(current_target(1), current_target(2), 'ro', 'MarkerSize', 20, 'LineWidth', 2);

% Callback de click
set(fig, 'WindowButtonDownFcn', @mouse_click_callback);

% Texto de estado
status_text = text(ax, field_xlim(1)+2, field_ylim(2)-2, 'Estado: Iniciando...', 'FontSize', 12, 'BackgroundColor', 'w', 'EdgeColor', 'k');

%% 3. BUCLE DE SIMULACIÓN
t_current = 0;
last_tic = tic;

X_ref_prev = [current_target; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];

try
    while isvalid(fig)
        % Gestionar tiempo
        dt_real = toc(last_tic);
        last_tic = tic;
        
        % Actualizar marcador visual del objetivo
        set(target_marker, 'XData', current_target(1), 'YData', current_target(2));
        set(target_circle, 'XData', current_target(1), 'YData', current_target(2));
        
        % 1. Calcular distancia y estado actual
        dist_to_target = norm(X(1:2) - current_target);
        v_abs = abs(X(7));
        
        % 2. Lógica de Control y "Parking"
        if dist_to_target < 0.6
            % --- ZONA DE LLEGADA (Parking) ---
            u_control = zeros(4,1); % Cortar motores (Zero Torque)
            
            % Si la velocidad ya es baja, forzar parada perfecta para evitar "drift"
            if v_abs < 0.1
                X(7:12) = 0;
            end
            
            status_str = sprintf(' T=%.1fs | LLEGADO (Click para mover) ', t_current);
            set(target_circle, 'Color', 'g');
        else
            % --- ZONA DE NAVEGACIÓN ---
            % Construir vector de referencia
            X_ref = zeros(12,1);
            X_ref(1) = current_target(1);
            X_ref(2) = current_target(2);
            
            % Calcular control difuso
            u_control = fuzzy_yaw_rate_controller(X, X_ref, X_ref_prev, dt, fis, params);
            
            status_str = sprintf(' T=%.1fs | Dist: %.2fm | V: %.2fm/s ', t_current, dist_to_target, v_abs);
            set(target_circle, 'Color', 'r');
        end
        
        set(status_text, 'String', status_str);
        
        % 3. Física (ode45 paso a paso)
        model_func = @(t,x) skid_steer_robot_model(t, x, u_control, params);
        [~, X_ode] = ode45(model_func, [0 dt], X);
        X = X_ode(end,:)';
        
        % Guardar estela visual
        X_trail = [X_trail, X(1)];
        Y_trail = [Y_trail, X(2)];
        if length(X_trail) > 500  % Limitar longitud de estela
            X_trail = X_trail(end-499:end);
            Y_trail = Y_trail(end-499:end);
        end
        set(trail_plot, 'XData', X_trail, 'YData', Y_trail);
        
        % Estado
        v_abs = abs(X(7));
        dist_to_target = norm(X(1:2) - current_target);
        
        if dist_to_target < 0.5 && v_abs < 0.1
            status_str = sprintf(' T=%.1fs | LLEGADO (Click para mover) ', t_current);
            set(target_circle, 'Color', 'g');
        else
            status_str = sprintf(' T=%.1fs | Dist: %.2fm | V: %.2fm/s ', t_current, dist_to_target, v_abs);
            set(target_circle, 'Color', 'r');
        end
        set(status_text, 'String', status_str);
        
        % 3. Actualizar Gráficos Robot 2D
        update_robot_graphics_2d(X, params, robot_body, heading_arrow);
        
        drawnow limitrate;
        
        t_current = t_current + dt;
        X_ref_prev = X_ref;
    end
catch ME
    if strcmp(ME.identifier, 'MATLAB:class:InvalidHandle')
        fprintf('Simulación terminada por el usuario.\n');
    else
        rethrow(ME);
    end
end

%% FUNCIÓN CALLBACK MOUSE
    function mouse_click_callback(src, ~)
        % Obtener coordenadas del click en el eje
        cp = get(gca, 'CurrentPoint');
        x_click = cp(1,1);
        y_click = cp(1,2);
        
        % Solo aceptamos clicks dentro de los límites
        if x_click >= field_xlim(1) && x_click <= field_xlim(2) && ...
                y_click >= field_ylim(1) && y_click <= field_ylim(2)
            current_target = [x_click; y_click];
            fprintf('► Nuevo objetivo: (%.2f, %.2f)\n', x_click, y_click);
        end
    end

%% FUNCIÓN DIBUJO ROBOT 2D
    function update_robot_graphics_2d(X, p, h_body, h_arrow)
        x = X(1); y = X(2); psi = X(6);
        
        % Dimensiones
        L = p.L; W = p.W;
        
        % Vértices del rectángulo del robot (local)
        % FL, FR, RR, RL
        pts_local = [
            L/2,  W/2;
            L/2, -W/2;
            -L/2, -W/2;
            -L/2,  W/2
            ]';
        
        % Matriz de rotación 2D
        R = [cos(psi), -sin(psi); sin(psi), cos(psi)];
        
        % Transformar al mundo
        pts_world = R * pts_local + [x; y];
        
        % Actualizar Patch
        set(h_body, 'XData', pts_world(1,:), 'YData', pts_world(2,:));
        
        % Actualizar Flecha
        set(h_arrow, 'XData', x, 'YData', y, ...
            'UData', cos(psi)*1.5, 'VData', sin(psi)*1.5); % Flecha un poco más larga
    end

end
