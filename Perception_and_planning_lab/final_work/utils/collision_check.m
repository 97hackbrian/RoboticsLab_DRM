function collision = collision_check(p1, p2, obstacles, margin)
% collision_check: Verifica colisión de un segmento con obstáculos
%
% ENTRADAS:
%   p1 - Punto inicial [x; y]
%   p2 - Punto final [x; y] (opcional, si solo se verifica un punto)
%   obstacles - Cell array de obstáculos (círculos/rectángulos)
%   margin - Margen de seguridad adicional (radio del robot)
%
% SALIDA:
%   collision - true si hay colisión, false si libre

if nargin < 4
    margin = 0;
end

% Si p2 no se proporciona, solo verificar punto p1
if nargin < 2 || isempty(p2)
    collision = point_collision(p1, obstacles, margin);
    return;
end

% Verificar colisión del segmento p1 -> p2
collision = segment_collision(p1, p2, obstacles, margin);

end

%% ════════════════════════════════════════════════════════════════════════
%  FUNCIONES AUXILIARES
%% ════════════════════════════════════════════════════════════════════════

function collision = point_collision(p, obstacles, margin)
% Verifica si un punto colisiona con algún obstáculo
collision = false;

for i = 1:length(obstacles)
    obs = obstacles{i};
    
    switch lower(obs.type)
        case 'circle'
            % Distancia al centro del círculo
            dist = norm(p - obs.center);
            if dist <= obs.radius + margin
                collision = true;
                return;
            end
            
        case 'rectangle'
            % Verificar si el punto está dentro del rectángulo expandido
            half_w = obs.width / 2 + margin;
            half_h = obs.height / 2 + margin;
            
            if abs(p(1) - obs.center(1)) <= half_w && ...
                    abs(p(2) - obs.center(2)) <= half_h
                collision = true;
                return;
            end
    end
end
end

function collision = segment_collision(p1, p2, obstacles, margin)
% Verifica si un segmento de línea colisiona con obstáculos
% Usa discretización del segmento para verificación robusta

collision = false;

% Longitud del segmento
seg_length = norm(p2 - p1);
if seg_length < 1e-6
    collision = point_collision(p1, obstacles, margin);
    return;
end

% Dirección unitaria
direction = (p2 - p1) / seg_length;

% Resolución de verificación (cada 0.1m)
resolution = 0.1;
num_checks = max(2, ceil(seg_length / resolution));

for i = 1:length(obstacles)
    obs = obstacles{i};
    
    switch lower(obs.type)
        case 'circle'
            % Método analítico para círculos (más preciso)
            if segment_circle_collision(p1, p2, obs.center, obs.radius + margin)
                collision = true;
                return;
            end
            
        case 'rectangle'
            % Discretización para rectángulos
            for j = 0:num_checks
                t = j / num_checks;
                p_test = p1 + t * (p2 - p1);
                
                half_w = obs.width / 2 + margin;
                half_h = obs.height / 2 + margin;
                
                if abs(p_test(1) - obs.center(1)) <= half_w && ...
                        abs(p_test(2) - obs.center(2)) <= half_h
                    collision = true;
                    return;
                end
            end
    end
end
end

function collision = segment_circle_collision(p1, p2, center, radius)
% Verificación analítica de colisión segmento-círculo
% Basado en proyección del centro al segmento

d = p2 - p1;
f = p1 - center;

a = dot(d, d);
b = 2 * dot(f, d);
c = dot(f, f) - radius^2;

discriminant = b^2 - 4*a*c;

if discriminant < 0
    collision = false;
    return;
end

discriminant = sqrt(discriminant);

t1 = (-b - discriminant) / (2*a);
t2 = (-b + discriminant) / (2*a);

% Verificar si alguna intersección está dentro del segmento [0, 1]
if (t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1)
    collision = true;
    return;
end

% Verificar si el segmento está completamente dentro del círculo
if t1 < 0 && t2 > 1
    collision = true;
    return;
end

collision = false;
end
