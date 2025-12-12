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

% Entrada 1: Error de heading (¡MÁS ESTRICTO! - Zona Zero reducida a ±15°)
% Esto hace que el controlador reaccione a errores pequeños ANTES de pasarse del setpoint
fis = addInput(fis, [-pi pi], 'Name', 'error_heading');
fis = addMF(fis, 'error_heading', 'trimf', [-pi -pi -pi/2], 'Name', 'N_Large');
fis = addMF(fis, 'error_heading', 'trimf', [-pi/2 -pi/4 -pi/18], 'Name', 'N_Med');   % Más sensible
fis = addMF(fis, 'error_heading', 'trimf', [-pi/12 0 pi/12], 'Name', 'Zero');        % ±15° (antes ±30°)
fis = addMF(fis, 'error_heading', 'trimf', [pi/18 pi/4 pi/2], 'Name', 'P_Med');      % Más sensible
fis = addMF(fis, 'error_heading', 'trimf', [pi/2 pi pi], 'Name', 'P_Large');

% Entrada 1b: Error PEQUEÑO (para correcciones finas durante acercamiento)
fis = addMF(fis, 'error_heading', 'trimf', [-pi/6 -pi/12 0], 'Name', 'N_Small');     % -30° a 0°
fis = addMF(fis, 'error_heading', 'trimf', [0 pi/12 pi/6], 'Name', 'P_Small');       % 0° a +30°

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


% Salida 2: Omega (MUY AGRESIVA para corrección rápida de orientación)
fis = addOutput(fis, [-4 4], 'Name', 'omega_desired');  % Rango aumentado a ±4 rad/s
fis = addMF(fis, 'omega_desired', 'trimf', [-4 -4 -2.5], 'Name', 'TurnLeft_Fast');
fis = addMF(fis, 'omega_desired', 'trimf', [-3.0 -2.0 -1.0], 'Name', 'TurnLeft_Med');   % Más agresivo
fis = addMF(fis, 'omega_desired', 'trimf', [-1.5 -1.0 -0.3], 'Name', 'TurnLeft_Slow');  % Más agresivo
fis = addMF(fis, 'omega_desired', 'trimf', [-0.5 -0.25 0], 'Name', 'TurnLeft_Tiny');    % NUEVO: Corrección fina
fis = addMF(fis, 'omega_desired', 'trimf', [-0.15 0 0.15], 'Name', 'Straight');         % Más estricto
fis = addMF(fis, 'omega_desired', 'trimf', [0 0.25 0.5], 'Name', 'TurnRight_Tiny');     % NUEVO: Corrección fina
fis = addMF(fis, 'omega_desired', 'trimf', [0.3 1.0 1.5], 'Name', 'TurnRight_Slow');    % Más agresivo
fis = addMF(fis, 'omega_desired', 'trimf', [1.0 2.0 3.0], 'Name', 'TurnRight_Med');     % Más agresivo
fis = addMF(fis, 'omega_desired', 'trimf', [2.5 4 4], 'Name', 'TurnRight_Fast');

%% REGLAS (Error_heading: 1=N_Large,2=N_Med,3=Zero,4=P_Med,5=P_Large,6=N_Small,7=P_Small)
%         (Omega: 1=TurnLeft_Fast,2=TurnLeft_Med,3=TurnLeft_Slow,4=TurnLeft_Tiny,
%                 5=Straight,6=TurnRight_Tiny,7=TurnRight_Slow,8=TurnRight_Med,9=TurnRight_Fast)
rules = [
    % 1. SEGURIDAD: Distancia muy cercana
    0 0 1  1 5  1 1;  % VeryClose -> Stop, Straight
    0 0 2  2 0  1 1;  % Close -> Slow
    
    % 2. TANK TURNS (Giro en su sitio para errores grandes)
    1 0 0  1 1  1 1;  % N_Large -> Stop, TurnLeft_Fast
    5 0 0  1 9  1 1;  % P_Large -> Stop, TurnRight_Fast
    2 0 0  1 2  1 1;  % N_Med -> Stop, TurnLeft_Med
    4 0 0  1 8  1 1;  % P_Med -> Stop, TurnRight_Med
    
    % 3. CORRECCIONES DURANTE ACERCAMIENTO (¡NUEVAS! - Clave para no pasarse)
    % Errores pequeños (6=N_Small, 7=P_Small) -> Corregir mientras avanza
    6 0 2  2 3  1 1;  % N_Small + Close -> Slow, TurnLeft_Slow (Corrige mientras frena)
    7 0 2  2 7  1 1;  % P_Small + Close -> Slow, TurnRight_Slow
    6 0 3  3 4  1 1;  % N_Small + Medium -> Medium speed, TurnLeft_Tiny
    7 0 3  3 6  1 1;  % P_Small + Medium -> Medium speed, TurnRight_Tiny
    6 0 4  3 3  1 1;  % N_Small + Far -> Medium speed, TurnLeft_Slow
    7 0 4  3 7  1 1;  % P_Small + Far -> Medium speed, TurnRight_Slow
    
    % 4. ALINEADO PERFECTO -> Avanzar recto
    3 0 0  0 5  1 1;  % Zero Error -> Straight
    
    % 5. VELOCIDAD DE CRUCERO (Solo si perfectamente alineado)
    3 0 3  3 5  1 1;  % Aligned + Medium Dist -> v=Medium, Straight
    3 0 4  4 5  1 1;  % Aligned + Far Dist -> v=Fast, Straight
    
    % 6. AMORTIGUAMIENTO (Damping para evitar overshoot de giro)
    3 1 0  0 6  0.8 1; % Error~0, Girando izq rapido -> TurnRight_Tiny (Frenar giro)
    3 3 0  0 4  0.8 1; % Error~0, Girando der rapido -> TurnLeft_Tiny (Frenar giro)
    
    % 7. CORRECCIONES PROGRESIVAS A MEDIA DISTANCIA
    2 0 3  2 2  1 1;  % N_Med + Medium -> Slow, TurnLeft_Med (Corregir antes de llegar)
    4 0 3  2 8  1 1;  % P_Med + Medium -> Slow, TurnRight_Med
    ];

fis = addRule(fis, rules);

fprintf('✓ FIS MEJORADO creado con %d reglas (granularidad fina)\n', length(fis.Rules));
fprintf('  Entradas: error_heading, derror_heading, distance\n');
fprintf('  Salidas: v_desired, omega_desired\n\n');

end
