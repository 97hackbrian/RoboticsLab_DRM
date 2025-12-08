function fis = fuzzy_yaw_rate_controller_setup()
% fuzzy_yaw_rate_controller_setup: FIS que controla velocidades (v, omega)
% Similar al PID del uniciclo, pero con lógica difusa
%
% SALIDA:
%   fis - Sistema de Inferencia Difuso con:
%         Entradas: error_heading, derror_heading, distance_to_goal
%         Salidas: v_desired (m/s), omega_desired (rad/s)

fprintf('Creando FIS para control de velocidades (tipo uniciclo)...\n');

%% CREAR FIS
fis = mamfis('Name', 'YawRateController');

%% ENTRADAS

% Entrada 1: Error de heading (MÁS ESTRECHO CERCA DE CERO)
fis = addInput(fis, [-pi pi], 'Name', 'error_heading');
fis = addMF(fis, 'error_heading', 'trimf', [-pi -pi -pi/2], 'Name', 'N_Large');
fis = addMF(fis, 'error_heading', 'trimf', [-2*pi/3 -pi/3 -pi/12], 'Name', 'N_Med'); % Ajustado
fis = addMF(fis, 'error_heading', 'trimf', [-pi/6 0 pi/6], 'Name', 'Zero');
fis = addMF(fis, 'error_heading', 'trimf', [pi/12 pi/3 2*pi/3], 'Name', 'P_Med'); % Ajustado
fis = addMF(fis, 'error_heading', 'trimf', [pi/2 pi pi], 'Name', 'P_Large');

% Entrada 2: Derivada de error (Damping)
fis = addInput(fis, [-2 2], 'Name', 'derror_heading');
fis = addMF(fis, 'derror_heading', 'trimf', [-2 -2 -0.2], 'Name', 'Neg');
fis = addMF(fis, 'derror_heading', 'trimf', [-0.5 0 0.5], 'Name', 'Zero');
fis = addMF(fis, 'derror_heading', 'trimf', [0.2 2 2], 'Name', 'Pos');

% Entrada 3: Distancia (Ajustada para frenado anticipado)
fis = addInput(fis, [0 20], 'Name', 'distance');
fis = addMF(fis, 'distance', 'trimf', [0 0 0.5], 'Name', 'VeryClose'); % Un poco más amplio
fis = addMF(fis, 'distance', 'trimf', [0.2 2 4], 'Name', 'Close');      % Empieza a frenar a 4m
fis = addMF(fis, 'distance', 'trimf', [3 8 15], 'Name', 'Medium');
fis = addMF(fis, 'distance', 'trimf', [8 20 20], 'Name', 'Far');

%% SALIDAS

% Salida 1: Velocidad (Ajuste "Muuuy Lento" para demostración)
fis = addOutput(fis, [0 1.5], 'Name', 'v_desired');
fis = addMF(fis, 'v_desired', 'trimf', [0 0 0.05], 'Name', 'Stop');
fis = addMF(fis, 'v_desired', 'trimf', [0 0.15 0.3], 'Name', 'Slow');   % ~ 0.15 m/s (Casi reptando)
fis = addMF(fis, 'v_desired', 'trimf', [0.2 0.6 1.0], 'Name', 'Medium'); % ~ 0.6 m/s
fis = addMF(fis, 'v_desired', 'trimf', [0.8 1.0 1.5], 'Name', 'Fast');   % ~ 1.0 m/s (Muy tranquilo)

%{


% Salida 1: Velocidad (Ajuste Fino)
fis = addOutput(fis, [0 2.5], 'Name', 'v_desired');
fis = addMF(fis, 'v_desired', 'trimf', [0 0 0.1], 'Name', 'Stop');
fis = addMF(fis, 'v_desired', 'trimf', [0 0.3 0.6], 'Name', 'Slow');   % ~ 0.3 m/s
fis = addMF(fis, 'v_desired', 'trimf', [0.5 1.0 1.5], 'Name', 'Medium'); % ~ 1.0 m/s
fis = addMF(fis, 'v_desired', 'trimf', [1.2 2.0 2.5], 'Name', 'Fast');   % ~ 2.0 m/s (Antes 3.0)

%}


% Salida 2: Omega (AUMENTADA para control más firme - "Tight")
fis = addOutput(fis, [-3 3], 'Name', 'omega_desired');
fis = addMF(fis, 'omega_desired', 'trimf', [-3 -3 -2.0], 'Name', 'TurnLeft_Fast');
fis = addMF(fis, 'omega_desired', 'trimf', [-2.5 -1.5 -0.8], 'Name', 'TurnLeft_Med');  % Aumentado (antes -1)
fis = addMF(fis, 'omega_desired', 'trimf', [-1.2 -0.8 -0.2], 'Name', 'TurnLeft_Slow'); % Aumentado (antes -0.3)
fis = addMF(fis, 'omega_desired', 'trimf', [-0.3 0 0.3], 'Name', 'Straight');
fis = addMF(fis, 'omega_desired', 'trimf', [0.2 0.8 1.2], 'Name', 'TurnRight_Slow');   % Aumentado
fis = addMF(fis, 'omega_desired', 'trimf', [0.8 1.5 2.5], 'Name', 'TurnRight_Med');    % Aumentado
fis = addMF(fis, 'omega_desired', 'trimf', [2.0 3 3], 'Name', 'TurnRight_Fast');

%% REGLAS
rules = [
    % 1. SEGURIDAD: Distancia
    0 0 1  1 4  1 1; % VeryClose -> Stop, Straight
    0 0 2  2 0  1 1; % Close -> Slow
    
    % 2. TANK TURNS (Giro en su sitio)
    1 0 0  1 1  1 1; % N_Large -> Stop, TurnLeft_Fast
    5 0 0  1 7  1 1; % P_Large -> Stop, TurnRight_Fast
    2 0 0  1 2  1 1; % N_Med -> Stop, TurnLeft_Med (Giro limpio)
    4 0 0  1 6  1 1; % P_Med -> Stop, TurnRight_Med (Giro limpio)
    
    % 3. CORRECCIONES FINAS (En movimiento)
    3 0 0  0 4  1 1; % Zero Error -> Straight
    
    % 4. VELOCIDAD DE CRUCERO (Solo si alineado)
    3 0 3  3 0  1 1; % Aligned + Medium Dist -> v=Medium
    3 0 4  4 0  1 1; % Aligned + Far Dist -> v=Fast
    
    % 5. AMORTIGUAMIENTO (Damping para evitar overshoot)
    % Si error es Zero pero derror es alto, contra-restar
    3 1 0  0 5  0.8 1; % Error~0, Girando izq rapido -> TurnRight_Slow (Frenar giro)
    3 3 0  0 3  0.8 1; % Error~0, Girando der rapido -> TurnLeft_Slow (Frenar giro)
    ];

fis = addRule(fis, rules);

fprintf('✓ FIS MEJORADO creado con %d reglas (granularidad fina)\n', length(fis.Rules));
fprintf('  Entradas: error_heading, derror_heading, distance\n');
fprintf('  Salidas: v_desired, omega_desired\n\n');

end
