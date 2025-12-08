function fis = fuzzy_controller_setup()
% fuzzy_controller_setup: Crea el sistema de inferencia difuso (FIS) para control de posición
%
% SALIDA:
%   fis - Sistema de inferencia difuso configurado
%
% DESCRIPCIÓN:
%   Este controlador difuso maneja:
%   - Seguimiento de posición X,Y
%   - Compensación de gravedad en pendientes
%   - Generación de torques para las 4 ruedas

fprintf('Creando Sistema de Inferencia Difuso...\n');

%% CREAR FIS
fis = mamfis('Name', 'PositionTrackingController');

%% DEFINIR ENTRADAS

% Entrada 1: Error de Posición en X (metros)
fis = addInput(fis, [-5 5], 'Name', 'error_x');
fis = addMF(fis, 'error_x', 'trimf', [-5 -5 -3], 'Name', 'NB'); % Negative Big
fis = addMF(fis, 'error_x', 'trimf', [-5 -2.5 0], 'Name', 'NM'); % Negative Medium
fis = addMF(fis, 'error_x', 'trimf', [-2 -0.5 0.5], 'Name', 'NS'); % Negative Small
fis = addMF(fis, 'error_x', 'trimf', [-0.5 0 0.5], 'Name', 'ZE'); % Zero
fis = addMF(fis, 'error_x', 'trimf', [-0.5 0.5 2], 'Name', 'PS'); % Positive Small
fis = addMF(fis, 'error_x', 'trimf', [0 2.5 5], 'Name', 'PM'); % Positive Medium
fis = addMF(fis, 'error_x', 'trimf', [3 5 5], 'Name', 'PB'); % Positive Big

% Entrada 2: Derivada del Error en X (m/s)
fis = addInput(fis, [-3 3], 'Name', 'derror_x');
fis = addMF(fis, 'derror_x', 'trimf', [-3 -3 -1.5], 'Name', 'NB');
fis = addMF(fis, 'derror_x', 'trimf', [-3 -1.5 0], 'Name', 'NM');
fis = addMF(fis, 'derror_x', 'trimf', [-1 -0.3 0.3], 'Name', 'NS');
fis = addMF(fis, 'derror_x', 'trimf', [-0.3 0 0.3], 'Name', 'ZE');
fis = addMF(fis, 'derror_x', 'trimf', [-0.3 0.3 1], 'Name', 'PS');
fis = addMF(fis, 'derror_x', 'trimf', [0 1.5 3], 'Name', 'PM');
fis = addMF(fis, 'derror_x', 'trimf', [1.5 3 3], 'Name', 'PB');

% Entrada 3: Error de Posición en Y (metros) - para control lateral
fis = addInput(fis, [-5 5], 'Name', 'error_y');
fis = addMF(fis, 'error_y', 'trimf', [-5 -5 -2], 'Name', 'NB');
fis = addMF(fis, 'error_y', 'trimf', [-3 -1.5 0], 'Name', 'NM');
fis = addMF(fis, 'error_y', 'trimf', [-1 -0.3 0.3], 'Name', 'NS');
fis = addMF(fis, 'error_y', 'trimf', [-0.3 0 0.3], 'Name', 'ZE');
fis = addMF(fis, 'error_y', 'trimf', [-0.3 0.3 1], 'Name', 'PS');
fis = addMF(fis, 'error_y', 'trimf', [0 1.5 3], 'Name', 'PM');
fis = addMF(fis, 'error_y', 'trimf', [2 5 5], 'Name', 'PB');

% Entrada 4: Ángulo Pitch (grados) - para compensación de gravedad
fis = addInput(fis, [-30 30], 'Name', 'pitch');
fis = addMF(fis, 'pitch', 'trimf', [-30 -30 -10], 'Name', 'DN'); % Down Steep
fis = addMF(fis, 'pitch', 'trimf', [-15 -5 0], 'Name', 'DS'); % Down Slight
fis = addMF(fis, 'pitch', 'trimf', [-5 0 5], 'Name', 'ZE'); % Zero
fis = addMF(fis, 'pitch', 'trimf', [0 5 15], 'Name', 'US'); % Up Slight
fis = addMF(fis, 'pitch', 'trimf', [10 30 30], 'Name', 'UN'); % Up Steep

%% DEFINIR SALIDAS

% Salida 1: Torque Longitudinal (promedio para las 4 ruedas)
fis = addOutput(fis, [-10 10], 'Name', 'torque_long');
fis = addMF(fis, 'torque_long', 'trimf', [-10 -10 -6], 'Name', 'NB');
fis = addMF(fis, 'torque_long', 'trimf', [-8 -4 0], 'Name', 'NM');
fis = addMF(fis, 'torque_long', 'trimf', [-3 -1 0], 'Name', 'NS');
fis = addMF(fis, 'torque_long', 'trimf', [-1 0 1], 'Name', 'ZE');
fis = addMF(fis, 'torque_long', 'trimf', [0 1 3], 'Name', 'PS');
fis = addMF(fis, 'torque_long', 'trimf', [0 4 8], 'Name', 'PM');
fis = addMF(fis, 'torque_long', 'trimf', [6 10 10], 'Name', 'PB');

% Salida 2: Torque Diferencial (para giro - diferencia entre izq/der)
% AUMENTADO el rango para mejorar capacidad de giro
fis = addOutput(fis, [-8 8], 'Name', 'torque_diff');
fis = addMF(fis, 'torque_diff', 'trimf', [-8 -8 -5], 'Name', 'NB');
fis = addMF(fis, 'torque_diff', 'trimf', [-6 -3 0], 'Name', 'NM');
fis = addMF(fis, 'torque_diff', 'trimf', [-2 -1 0], 'Name', 'NS');
fis = addMF(fis, 'torque_diff', 'trimf', [-0.5 0 0.5], 'Name', 'ZE');
fis = addMF(fis, 'torque_diff', 'trimf', [0 1 2], 'Name', 'PS');
fis = addMF(fis, 'torque_diff', 'trimf', [0 3 6], 'Name', 'PM');
fis = addMF(fis, 'torque_diff', 'trimf', [5 8 8], 'Name', 'PB');

%% REGLAS DIFUSAS

% Nota: Rules format: [Input1 Input2 Input3 Input4 Output1 Output2 Weight Operator]
% Operator: 1=AND, 2=OR

rules = [
    % CONTROL LONGITUDINAL (basado en error_x, derror_x, pitch)
    % Si error grande positivo → acelerar
    7 0 0 0  7 0  1 1;  % error_x=PB → T_long=PB
    6 0 0 0  6 0  1 1;  % error_x=PM → T_long=PM
    5 0 0 0  5 0  1 1;  % error_x=PS → T_long=PS
    
    % Si error grande negativo → frenar/reversa
    1 0 0 0  1 0  1 1;  % error_x=NB → T_long=NB
    2 0 0 0  2 0  1 1;  % error_x=NM → T_long=NM
    
    % Error cero → mantener
    4 4 0 0  4 0  1 1;  % error_x=ZE, derror_x=ZE → T_long=ZE
    
    % Compensación por velocidad (amortiguamiento)
    0 7 0 0  2 0  0.8 1;  % derror_x=PB → reducir torque
    0 1 0 0  6 0  0.8 1;  % derror_x=NB → aumentar torque
    
    % COMPENSACIÓN DE GRAVEDAD
    % Subida (pitch positivo) → más torque
    5 0 0 4  6 0  1 1;  % error_x=PS, pitch=US → T_long=PM
    5 0 0 5  7 0  1 1;  % error_x=PS, pitch=UN → T_long=PB
    4 0 0 4  5 0  1 1;  % error_x=ZE, pitch=US → T_long=PS (mantener en subida)
    
    % Bajada (pitch negativo) → menos torque/frenar
    4 0 0 2  3 0  1 1;  % error_x=ZE, pitch=DS → T_long=NS (frenar en bajada)
    3 0 0 1  2 0  1 1;  % error_x=NS, pitch=DN → T_long=NM (frenar fuerte)
    
    % CONTROL LATERAL (basado en error_y) - MEJORADO con más reglas
    % Error Y positivo → girar a la derecha (diferencial positivo)
    0 0 7 0  0 7  1 1;  % error_y=PB → T_diff=PB
    0 0 6 0  0 6  1 1;  % error_y=PM → T_diff=PM
    0 0 5 0  0 5  1 1;  % error_y=PS → T_diff=PS
    
    % Error Y negativo → girar a la izquierda (diferencialnegativo)
    0 0 1 0  0 1  1 1;  % error_y=NB → T_diff=NB
    0 0 2 0  0 2  1 1;  % error_y=NM → T_diff=NM
    0 0 3 0  0 3  1 1;  % error_y=NS → T_diff=NS
    
    % Error Y cero → sin giro
    0 0 4 0  0 4  1 1;  % error_y=ZE → T_diff=ZE
    ];

fis = addRule(fis, rules);

%% CONFIGURACIÓN DE DEFUZZIFICATION
fis.DefuzzificationMethod = 'centroid'; % Método del centroide

fprintf('✓ FIS creado con %d reglas\n', length(fis.Rules));
fprintf('  Entradas: error_x, derror_x, error_y, pitch\n');
fprintf('  Salidas: torque_long, torque_diff\n\n');

end
