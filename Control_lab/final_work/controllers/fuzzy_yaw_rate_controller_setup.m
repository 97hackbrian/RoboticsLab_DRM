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

% Entrada 1: Error de heading (ángulo al objetivo) - rad
fis = addInput(fis, [-pi pi], 'Name', 'error_heading');
fis = addMF(fis, 'error_heading', 'trimf', [-pi -pi -pi/2], 'Name', 'N_Large');
fis = addMF(fis, 'error_heading', 'trimf', [-pi -pi/4 0], 'Name', 'N_Med');
fis = addMF(fis, 'error_heading', 'trimf', [-pi/6 0 pi/6], 'Name', 'Zero');
fis = addMF(fis, 'error_heading', 'trimf', [0 pi/4 pi], 'Name', 'P_Med');
fis = addMF(fis, 'error_heading', 'trimf', [pi/2 pi pi], 'Name', 'P_Large');

% Entrada 2: Derivada de error de heading - rad/s
fis = addInput(fis, [-2 2], 'Name', 'derror_heading');
fis = addMF(fis, 'derror_heading', 'trimf', [-2 -2 -0.5], 'Name', 'Neg');
fis = addMF(fis, 'derror_heading', 'trimf', [-1 0 1], 'Name', 'Zero');
fis = addMF(fis, 'derror_heading', 'trimf', [0.5 2 2], 'Name', 'Pos');

% Entrada 3: Distancia al objetivo - metros
fis = addInput(fis, [0 20], 'Name', 'distance');
fis = addMF(fis, 'distance', 'trimf', [0 0 0.5], 'Name', 'VeryClose');
fis = addMF(fis, 'distance', 'trimf', [0.3 1 2], 'Name', 'Close');
fis = addMF(fis, 'distance', 'trimf', [1.5 5 10], 'Name', 'Medium');
fis = addMF(fis, 'distance', 'trimf', [8 20 20], 'Name', 'Far');

%% SALIDAS

% Salida 1: Velocidad lineal deseada - m/s (AUMENTADA para completar trayectoria)
fis = addOutput(fis, [0 3], 'Name', 'v_desired');
fis = addMF(fis, 'v_desired', 'trimf', [0 0 0], 'Name', 'Stop');
fis = addMF(fis, 'v_desired', 'trimf', [0 0.5 1.0], 'Name', 'Slow');
fis = addMF(fis, 'v_desired', 'trimf', [0.8 1.5 2.2], 'Name', 'Medium');
fis = addMF(fis, 'v_desired', 'trimf', [1.8 3.0 3.0], 'Name', 'Fast');

% Salida 2: Velocidad angular deseada - rad/s
fis = addOutput(fis, [-3 3], 'Name', 'omega_desired');
fis = addMF(fis, 'omega_desired', 'trimf', [-3 -3 -1.5], 'Name', 'TurnLeft_Fast');
fis = addMF(fis, 'omega_desired', 'trimf', [-2 -0.8 -0.2], 'Name', 'TurnLeft_Slow');
fis = addMF(fis, 'omega_desired', 'trimf', [-0.3 0 0.3], 'Name', 'Straight');
fis = addMF(fis, 'omega_desired', 'trimf', [0.2 0.8 2], 'Name', 'TurnRight_Slow');
fis = addMF(fis, 'omega_desired', 'trimf', [1.5 3 3], 'Name', 'TurnRight_Fast');

%% REGLAS DIFUSAS
% Formato: [e_heading, de_heading, distance] => [v, omega]

rules = [
    % Si estamos MUY cerca, detenerse
    0 0 1  1 3  1 1;  % distance=VeryClose => v=Stop, omega=Straight
    
    % Si distance=Close, ir despacio
    0 0 2  2 0  1 1;  % distance=Close => v=Slow, omega=(depends on heading)
    
    % Reglas de velocidad angular basadas en error de heading
    % Error grande negativo (objetivo a la izquierda)
    1 0 0  0 1  1 1;  % e_heading=N_Large => omega=TurnLeft_Fast
    2 0 0  0 2  1 1;  % e_heading=N_Med => omega=TurnLeft_Slow
    
    % Error grande positivo (objetivo a la derecha)
    5 0 0  0 5  1 1;  % e_heading=P_Large => omega=TurnRight_Fast
    4 0 0  0 4  1 1;  % e_heading=P_Med => omega=TurnRight_Slow
    
    % Heading correcto
    3 0 0  0 3  1 1;  % e_heading=Zero => omega=Straight
    
    % Si el error de ángulo es MEDIO o GRANDE, DETENERSE para girar (Tank Turn)
    1 0 0  1 1  1 1;  % e_heading=N_Large -> v=STOP
    5 0 0  1 5  1 1;  % e_heading=P_Large -> v=STOP
    2 0 0  1 2  1 1;  % e_heading=N_Med   -> v=STOP (Antes Slow, ahora Stop para evitar bucles)
    4 0 0  1 4  1 1;  % e_heading=P_Med   -> v=STOP (Antes Slow, ahora Stop para evitar bucles)
    
    % Si heading está bien (Zero) y lejos, entonces correr
    3 0 3  3 3  1 1;  % e_heading=Zero & dist=Medium -> v=Medium, omega=Straight
    3 0 4  4 3  1 1;  % e_heading=Zero & dist=Far -> v=Fast, omega=Straight
    
    % Reglas de seguridad básicas (ya existentes pero refinadas)
    0 0 1  1 3  1 1;  % distance=VeryClose -> Stop
    0 0 2  2 0  1 1;  % distance=Close -> Slow
    ];

fis = addRule(fis, rules);

fprintf('✓ FIS creado con %d reglas\n', length(fis.Rules));
fprintf('  Entradas: error_heading, derror_heading, distance\n');
fprintf('  Salidas: v_desired, omega_desired\n\n');

end
