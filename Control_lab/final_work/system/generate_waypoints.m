function waypoints = generate_waypoints(trajectory_type, params)
% generate_waypoints: Genera puntos discretos (waypoints) para navegación
%
% ENTRADAS:
%   trajectory_type - Tipo: 'line', 'circle', 'square', 's_curve'
%   params          - Parámetros de la trayectoria
%
% SALIDA:
%   waypoints - Matriz [Nx2] de puntos [x, y] a alcanzar

switch lower(trajectory_type)
    case 'line'
        % Línea recta: solo 2 puntos (inicio y fin)
        x_start = 0;
        x_end = 10;
        waypoints = [x_start, 0; x_end, 0];
        
    case 'square'
        % Cuadrado: 4 vértices + regreso al origen
        side = params.side;
        waypoints = [
            0, 0;        % Inicio
            side, 0;     % Esquina 1
            side, side;  % Esquina 2
            0, side;     % Esquina 3
            0, 0         % Regreso al inicio
            ];
        
    case 'circle'
        % Círculo: N puntos distribuidos uniformemente
        R = params.radius;
        N_points = params.num_points; % Número de waypoints
        
        waypoints = zeros(N_points + 1, 2);
        for i = 1:N_points
            theta = 2*pi * (i-1) / N_points;
            waypoints(i, 1) = R * cos(theta) + R; % Offset para empezar en (0,0)
            waypoints(i, 2) = R * sin(theta);
        end
        % Cerrar el círculo
        waypoints(N_points + 1, :) = waypoints(1, :);
        
    case 's_curve'
        % S-curve: Serie de puntos con transición suave lateral
        N_points = params.num_points;
        x_end = 15;
        y_end = 3;
        
        waypoints = zeros(N_points, 2);
        for i = 1:N_points
            t_norm = (i-1) / (N_points-1);
            waypoints(i, 1) = t_norm * x_end;
            
            % Sigmoid para transición suave
            k = 10;
            sigmoid = 1 / (1 + exp(-k*(t_norm - 0.5)));
            waypoints(i, 2) = sigmoid * y_end;
        end
        
    otherwise
        error('Tipo de trayectoria no reconocido: %s', trajectory_type);
end

fprintf('✓ Generados %d waypoints para "%s"\n', size(waypoints, 1), trajectory_type);

end
