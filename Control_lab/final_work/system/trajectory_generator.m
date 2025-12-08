function [t, X_ref] = trajectory_generator(trajectory_type, t_span, dt, params)
% trajectory_generator: Genera trayectorias de referencia para tracking control
%
% ENTRADAS:
%   trajectory_type - Tipo de trayectoria: 'line', 'circle', 's_curve', 'step'
%   t_span          - Vector [t_start, t_end] (segundos)
%   dt              - Paso de tiempo (s)
%   params          - (Opcional) Parámetros adicionales de la trayectoria
%
% SALIDAS:
%   t    - Vector de tiempo [Nx1]
%   X_ref - Matriz de estados de referencia [Nx12]
%           Solo x, y son relevantes; otros estados pueden ser cero o derivados

% Vector de tiempo
t = (t_span(1):dt:t_span(2))';
N = length(t);

% Inicializar matriz de referencias (12 estados)
X_ref = zeros(N, 12);

switch lower(trajectory_type)
    case 'line'
        % Línea recta de (0,0) a (10,0)
        x_start = 0;
        x_end = 10;
        y_start = 0;
        y_end = 0;
        
        % Interpolación suave (rampa con saturación)
        T_total = t_span(2) - t_span(1);
        for i = 1:N
            t_norm = (t(i) - t_span(1)) / T_total; % Normalizado [0,1]
            X_ref(i,1) = x_start + (x_end - x_start) * t_norm;
            X_ref(i,2) = y_start + (y_end - y_start) * t_norm;
        end
        
    case 'circle'
        % Círculo de radio R en el plano XY
        if nargin < 4 || ~isfield(params, 'radius')
            R = 3; % Radio por defecto
        else
            R = params.radius;
        end
        
        if nargin < 4 || ~isfield(params, 'period')
            T_period = 10; % Período de revolución (s)
        else
            T_period = params.period;
        end
        
        omega = 2*pi / T_period; % Velocidad angular
        
        for i = 1:N
            theta = omega * (t(i) - t_span(1));
            X_ref(i,1) = R * cos(theta) + R; % Offset para que empiece en (0,0)
            X_ref(i,2) = R * sin(theta);
        end
        
    case 's_curve'
        % Curva en S (maniobra de cambio de carril)
        x_start = 0;
        x_end = 15;
        y_start = 0;
        y_end = 3; % Cambio lateral de 3 metros
        
        T_total = t_span(2) - t_span(1);
        
        for i = 1:N
            t_norm = (t(i) - t_span(1)) / T_total;
            
            % Interpolación X lineal
            X_ref(i,1) = x_start + (x_end - x_start) * t_norm;
            
            % Interpolación Y con función sigmoide suave
            % y(t) = y_end * (1 / (1 + exp(-k*(t_norm - 0.5))))
            k = 10; % Factor de suavidad
            sigmoid = 1 / (1 + exp(-k*(t_norm - 0.5)));
            X_ref(i,2) = y_start + (y_end - y_start) * sigmoid;
        end
        
    case 'step'
        % Escalón en X (útil para análisis de respuesta)
        x_step = 5; % Escalón de 5 metros
        
        for i = 1:N
            if t(i) < t_span(1) + 0.5
                X_ref(i,1) = 0;
            else
                X_ref(i,1) = x_step;
            end
            X_ref(i,2) = 0;
        end
        
    case 'square'
        % Trayectoria cuadrada CON ESQUINAS SUAVIZADAS
        side = 4; % Lado del cuadrado (metros)
        T_total = t_span(2) - t_span(1);
        T_side = T_total / 4; % Tiempo por lado
        corner_radius = 0.5; % Radio de suavizado en las esquinas (metros)
        
        for i = 1:N
            t_rel = t(i) - t_span(1);
            
            % Determinar en qué lado estamos
            side_num = floor(t_rel / T_side);
            t_in_side = mod(t_rel, T_side);
            progress = t_in_side / T_side; % [0, 1] dentro del lado actual
            
            % Definir puntos de esquina
            corners = [0, 0; side, 0; side, side; 0, side; 0, 0];
            
            if side_num >= 4
                % Completado - quedarse en inicio
                X_ref(i,1) = 0;
                X_ref(i,2) = 0;
            else
                % Puntos inicial y final del segmento actual
                p_start = corners(side_num + 1, :);
                p_end = corners(side_num + 2, :);
                
                % Interpolación suave usando función sigmoidea en las transiciones
                % Esto crea esquinas redondeadas naturalmente
                smooth_factor = 8; % Mayor = transición más brusca
                
                % Aplicar suavizado cerca de las esquinas (primero y último 20% del lado)
                if progress < 0.2
                    % Saliendo de esquina - suavizar inicio
                    alpha = 0.5 * (1 + tanh(smooth_factor * (progress - 0.1) / 0.1));
                elseif progress > 0.8
                    % Entrando a esquina - suavizar final
                    alpha = 0.5 * (1 - tanh(smooth_factor * (progress - 0.9) / 0.1)) + 0.5;
                else
                    % Sección media - movimiento lineal
                    alpha = progress;
                end
                
                % Interpolar posición
                X_ref(i,1) = p_start(1) + alpha * (p_end(1) - p_start(1));
                X_ref(i,2) = p_start(2) + alpha * (p_end(2) - p_start(2));
            end
        end
        
    otherwise
        error('Tipo de trayectoria no reconocido: %s', trajectory_type);
end

fprintf('✓ Trayectoria "%s" generada: %d puntos, T = %.1f s\n', ...
    trajectory_type, N, t_span(2) - t_span(1));

end
