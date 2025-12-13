function map = environment_map(scenario)
% environment_map: Configuración del entorno para path planning
%
% ENTRADA:
%   scenario - (opcional) Nombre del escenario predefinido:
%              'empty', 'simple', 'maze', 'random', 'narrow'
%
% SALIDA:
%   map - Estructura con:
%         .x_min, .x_max, .y_min, .y_max - Límites del mapa
%         .obstacles - Cell array de obstáculos
%         .start - Posición inicial [x; y]
%         .goal - Posición objetivo [x; y]
%         .robot_radius - Radio del robot para margen de colisión

if nargin < 1
    scenario = 'simple';
end

%% Configuración Base del Mapa
map.x_min = -2;
map.x_max = 20;
map.y_min = -2;
map.y_max = 20;

% Radio del robot para margen de seguridad en colisiones
map.robot_radius = 0.5;  % metros (considerando dimensiones del robot)

% Posiciones por defecto
map.start = [0; 0];
map.goal = [15; 15];

%% Escenarios Predefinidos
switch lower(scenario)
    case 'empty'
        % Entorno vacío para pruebas básicas
        map.obstacles = {};
        map.goal = [18; 18];
        
    case 'simple'
        % Obstáculos simples para demostración
        map.obstacles = {
            struct('type', 'circle', 'center', [8; 8], 'radius', 2.0)
            struct('type', 'circle', 'center', [5; 12], 'radius', 1.5)
            struct('type', 'circle', 'center', [12; 5], 'radius', 1.5)
            struct('type', 'rectangle', 'center', [3; 5], 'width', 2, 'height', 4)
            };
        
    case 'maze'
        % Laberinto con paredes
        map.obstacles = {
            % Paredes verticales
            struct('type', 'rectangle', 'center', [5; 10], 'width', 1, 'height', 12)
            struct('type', 'rectangle', 'center', [10; 8], 'width', 1, 'height', 10)
            struct('type', 'rectangle', 'center', [15; 12], 'width', 1, 'height', 8)
            % Paredes horizontales
            struct('type', 'rectangle', 'center', [7.5; 4], 'width', 6, 'height', 1)
            struct('type', 'rectangle', 'center', [12.5; 14], 'width', 6, 'height', 1)
            };
        map.goal = [18; 2];
        
    case 'labyrinth'
        % LABERINTO COMPLETO con múltiples rutas posibles
        map.x_min = -1;
        map.x_max = 22;
        map.y_min = -1;
        map.y_max = 22;
        
        wall_thickness = 0.2;
        
        map.obstacles = {
            % === PAREDES EXTERIORES (con aberturas) ===
            % Pared izquierda
            struct('type', 'rectangle', 'center', [0; 5], 'width', wall_thickness, 'height', 10)
            struct('type', 'rectangle', 'center', [0; 17], 'width', wall_thickness, 'height', 6)
            % Pared derecha
            struct('type', 'rectangle', 'center', [20; 8], 'width', wall_thickness, 'height', 14)
            % Pared superior
            struct('type', 'rectangle', 'center', [5; 20], 'width', 5, 'height', wall_thickness)
            struct('type', 'rectangle', 'center', [17; 20], 'width', 3, 'height', wall_thickness)
            % Pared inferior
            struct('type', 'rectangle', 'center', [12; 0], 'width', 5, 'height', wall_thickness)
            
            % === LABERINTO INTERNO - Fila 1 (abajo) ===
            struct('type', 'rectangle', 'center', [4; 3], 'width', 2, 'height', wall_thickness)
            struct('type', 'rectangle', 'center', [4; 6], 'width', wall_thickness, 'height', 6)
            struct('type', 'rectangle', 'center', [8; 4.5], 'width', wall_thickness, 'height', 3)
            
            % === LABERINTO INTERNO - Fila 2 ===
            struct('type', 'rectangle', 'center', [12; 4], 'width', 3, 'height', wall_thickness)
            struct('type', 'rectangle', 'center', [16; 6], 'width', wall_thickness, 'height', 4)
            struct('type', 'rectangle', 'center', [12; 7], 'width', 5, 'height', wall_thickness)
            
            % === LABERINTO INTERNO - Fila 3 (centro) ===
            struct('type', 'rectangle', 'center', [3; 10], 'width', 4, 'height', wall_thickness)
            struct('type', 'rectangle', 'center', [8; 10], 'width', wall_thickness, 'height', 6)
            struct('type', 'rectangle', 'center', [11; 10], 'width', 4, 'height', wall_thickness)
            struct('type', 'rectangle', 'center', [16; 10], 'width', 3, 'height', wall_thickness)
            
            % === LABERINTO INTERNO - Fila 4 ===
            struct('type', 'rectangle', 'center', [4; 13], 'width', 3, 'height', wall_thickness)
            struct('type', 'rectangle', 'center', [12; 13], 'width', wall_thickness, 'height', 6)
            struct('type', 'rectangle', 'center', [16; 14], 'width', 3, 'height', wall_thickness)
            
            % === LABERINTO INTERNO - Fila 5 (arriba) ===
            struct('type', 'rectangle', 'center', [4; 16], 'width', wall_thickness, 'height', 6)
            struct('type', 'rectangle', 'center', [7; 17], 'width', 4, 'height', wall_thickness)
            struct('type', 'rectangle', 'center', [16; 17], 'width', wall_thickness, 'height', 6)
            };
        
        % Inicio esquina inferior izquierda, meta esquina superior derecha
        map.start = [1; 1];
        map.goal = [19; 19];
        
    case 'narrow'
        % Pasajes estrechos
        map.obstacles = {
            struct('type', 'rectangle', 'center', [8; 5], 'width', 14, 'height', 2)
            struct('type', 'rectangle', 'center', [8; 15], 'width', 14, 'height', 2)
            % Dejar pasillos estrechos
            struct('type', 'circle', 'center', [5; 10], 'radius', 2)
            struct('type', 'circle', 'center', [11; 10], 'radius', 2)
            };
        map.start = [2; 2];
        map.goal = [16; 18];
        
    case 'random'
        % Obstáculos aleatorios
        rng(42);  % Semilla para reproducibilidad
        num_obstacles = 8;
        map.obstacles = cell(num_obstacles, 1);
        
        for i = 1:num_obstacles
            cx = 3 + rand() * 14;  % Centro X entre 3 y 17
            cy = 3 + rand() * 14;  % Centro Y entre 3 y 17
            
            % Evitar colocar obstáculos cerca del inicio o meta
            if norm([cx; cy] - map.start) < 2 || norm([cx; cy] - map.goal) < 2
                cx = cx + 3;
                cy = cy + 3;
            end
            
            if rand() > 0.5
                % Círculo
                map.obstacles{i} = struct('type', 'circle', ...
                    'center', [cx; cy], ...
                    'radius', 0.8 + rand() * 1.2);
            else
                % Rectángulo
                map.obstacles{i} = struct('type', 'rectangle', ...
                    'center', [cx; cy], ...
                    'width', 1 + rand() * 2, ...
                    'height', 1 + rand() * 2);
            end
        end
        
    case 'challenge'
        % Escenario desafiante con múltiples tipos de obstáculos
        map.obstacles = {
            % Zona central densa
            struct('type', 'circle', 'center', [10; 10], 'radius', 2.5)
            struct('type', 'circle', 'center', [7; 7], 'radius', 1.2)
            struct('type', 'circle', 'center', [13; 7], 'radius', 1.2)
            struct('type', 'circle', 'center', [7; 13], 'radius', 1.2)
            struct('type', 'circle', 'center', [13; 13], 'radius', 1.2)
            % Barreras laterales
            struct('type', 'rectangle', 'center', [3; 10], 'width', 1.5, 'height', 8)
            struct('type', 'rectangle', 'center', [17; 10], 'width', 1.5, 'height', 8)
            % Obstáculos adicionales
            struct('type', 'circle', 'center', [5; 3], 'radius', 1)
            struct('type', 'circle', 'center', [15; 17], 'radius', 1)
            };
        map.start = [1; 1];
        map.goal = [18; 18];
        
    case 'extreme'
        % MAPA EXTREMO: Muchos obstáculos, inicio y meta en extremos opuestos
        map.x_min = -2;
        map.x_max = 25;
        map.y_min = -2;
        map.y_max = 25;
        
        map.obstacles = {
            % Fila 1 - Barrera inferior
            struct('type', 'circle', 'center', [5; 3], 'radius', 1.5)
            struct('type', 'circle', 'center', [10; 4], 'radius', 1.8)
            struct('type', 'rectangle', 'center', [16; 3], 'width', 3, 'height', 2)
            struct('type', 'circle', 'center', [21; 4], 'radius', 1.5)
            
            % Fila 2 - Obstáculos medios-bajos
            struct('type', 'rectangle', 'center', [3; 8], 'width', 2, 'height', 4)
            struct('type', 'circle', 'center', [8; 7], 'radius', 2.0)
            struct('type', 'circle', 'center', [13; 8], 'radius', 1.5)
            struct('type', 'rectangle', 'center', [18; 7], 'width', 4, 'height', 2)
            
            % Fila 3 - Zona central densa
            struct('type', 'circle', 'center', [5; 12], 'radius', 1.8)
            struct('type', 'rectangle', 'center', [10; 12], 'width', 3, 'height', 3)
            struct('type', 'circle', 'center', [15; 11], 'radius', 2.2)
            struct('type', 'circle', 'center', [20; 12], 'radius', 1.5)
            
            % Fila 4 - Obstáculos medios-altos
            struct('type', 'rectangle', 'center', [3; 16], 'width', 2, 'height', 3)
            struct('type', 'circle', 'center', [8; 17], 'radius', 1.5)
            struct('type', 'circle', 'center', [12; 16], 'radius', 1.8)
            struct('type', 'rectangle', 'center', [17; 17], 'width', 3, 'height', 2)
            struct('type', 'circle', 'center', [22; 16], 'radius', 1.2)
            
            % Fila 5 - Barrera superior
            struct('type', 'circle', 'center', [5; 21], 'radius', 1.5)
            struct('type', 'rectangle', 'center', [10; 20], 'width', 2, 'height', 3)
            struct('type', 'circle', 'center', [15; 21], 'radius', 1.8)
            struct('type', 'circle', 'center', [20; 20], 'radius', 1.5)
            };
        
        % Inicio en esquina inferior izquierda, meta en esquina superior derecha
        map.start = [0; 0];
        map.goal = [23; 23];
        
    otherwise
        warning('Escenario "%s" no reconocido. Usando "simple".', scenario);
        map = environment_map('simple');
        return;
end

fprintf('► Mapa cargado: "%s" con %d obstáculos\n', scenario, length(map.obstacles));
fprintf('  Inicio: (%.1f, %.1f) → Meta: (%.1f, %.1f)\n', ...
    map.start(1), map.start(2), map.goal(1), map.goal(2));

end
