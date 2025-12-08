function waypoints = generate_waypoints(trajectory_type, params)
% generate_waypoints: Genera puntos con CURVAS SUAVES en esquinas
%
% ENTRADAS:
%   trajectory_type - Tipo: 'line', 'circle', 'square', 's_curve'
%   params          - Parámetros de la trayectoria
%
% SALIDA:
%   waypoints - Matriz [Nx2] de puntos [x, y] con transiciones suaves

switch lower(trajectory_type)
    case 'line'
        % Línea recta: solo 2 puntos
        waypoints = [0, 0; 10, 0];
        
    case 'square'
        % Cuadrado SIMPLE (solo esquinas) para evitar lazos raros
        side = params.side;
        
        % Definir explícitamente los 4 puntos + cierre
        waypoints = [
            0, 0;       % Inicio
            side, 0;    % Esquina 1 (X+)
            side, side; % Esquina 2 (X+, Y+)
            0, side;    % Esquina 3 (Y+)
            0, 0        % Cierre (Inicio)
            ];
        
    case 'circle'
        % Círculo: múltiples waypoints
        R = params.radius;
        N_points = params.num_points;
        
        waypoints = zeros(N_points + 1, 2);
        for i = 1:N_points
            theta = 2*pi * (i-1) / N_points;
            waypoints(i, 1) = R * cos(theta) + R;
            waypoints(i, 2) = R * sin(theta);
        end
        waypoints(N_points + 1, :) = waypoints(1, :);
        
    case 's_curve'
        % S-curve: waypoints con suavizado
        N_points = params.num_points;
        x_end = 15;
        y_end = 3;
        
        waypoints = zeros(N_points, 2);
        for i = 1:N_points
            t_norm = (i-1) / (N_points-1);
            waypoints(i, 1) = t_norm * x_end;
            sigmoid = 1 / (1 + exp(-10*(t_norm - 0.5)));
            waypoints(i, 2) = sigmoid * y_end;
        end
        
    otherwise
        error('Tipo no reconocido: %s', trajectory_type);
end

fprintf('✓ Generados %d waypoints para "%s"\n', size(waypoints, 1), trajectory_type);

end
