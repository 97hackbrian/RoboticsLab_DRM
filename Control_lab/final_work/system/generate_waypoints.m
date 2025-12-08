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
        % Cuadrado con CURVAS SUAVES automáticas
        side = params.side;
        corner_radius = min(side * 0.25, 1.5); % 25% del lado o 1.5m máximo
        
        % Esquinas originales
        corners = [0, 0; side, 0; side, side; 0, side; 0, 0];
        
        % Generar waypoints con curvas
        waypoints = [];
        num_curve_points = 3; % Puntos por curva
        
        for i = 1:(size(corners,1)-1)
            p_current = corners(i,:);
            p_next = corners(i+1,:);
            
            % Vector dirección actual
            dir = p_next - p_current;
            len = norm(dir);
            dir_norm = dir / len;
            
            if i == 1
                % Primer segmento: desde inicio
                waypoints = [waypoints; p_current];
                % Añadir punto antes de la esquina
                waypoints = [waypoints; p_current + dir_norm * (len - corner_radius)];
            else
                % Segmentos intermedios: curva + recta
                % Puntos de curva (transición suave)
                for j = 1:num_curve_points
                    alpha = j / (num_curve_points + 1);
                    waypoints = [waypoints; p_current + dir_norm * (corner_radius * alpha)];
                end
                
                % Punto antes de siguiente esquina
                if i < size(corners,1)-1
                    waypoints = [waypoints; p_current + dir_norm * (len - corner_radius)];
                else
                    % Último punto (regreso al inicio)
                    waypoints = [waypoints; p_next];
                end
            end
        end
        
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
