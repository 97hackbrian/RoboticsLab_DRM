function params = robot_parameters()
% robot_parameters: Configuración centralizada de parámetros del robot
%
% SALIDA:
%   params - Estructura con todos los parámetros físicos, EKF y controlador

%% PARÁMETROS FÍSICOS DEL ROBOT
params.m = 20;                      % Masa total (kg)
params.I = diag([0.5, 0.8, 0.8]);   % Tensor de inercia [Ix, Iy, Iz] (kg·m²)
params.r_wheel = 0.1;               % Radio de rueda (m)
params.L = 0.5;                     % Distancia entre ejes (m)
params.W = 0.4;                     % Ancho del robot (m)
params.h_chassis = 0.05;            % Altura del chasis (m)

%% PARÁMETROS DE TERRENO Y FRICCIÓN
params.mu_static = 0.3;             % Coeficiente de fricción estática REDUCIDO
params.mu_kinetic = 0.25;           % Coeficiente de fricción cinética REDUCIDO
params.mu_lateral = 0.4;            % Fricción lateral (Skid-Steer) REDUCIDO
params.Cd = 0.1;                    % Coeficiente de fricción viscosa

%% PARÁMETROS DEL EKF (Extended Kalman Filter)
% Matriz de covarianza de ruido de proceso Q (15x15)
params.Q_ekf = diag([...
    0.01^2 * ones(1,3), ...         % Ruido de posición (m²)
    0.02^2 * ones(1,3), ...         % Ruido de ángulos (rad²)
    0.1^2 * ones(1,3), ...          % Ruido de velocidades lineales (m²/s²)
    0.05^2 * ones(1,3), ...         % Ruido de velocidades angulares (rad²/s²)
    0.001^2 * ones(1,3)]);          % Ruido de bias giroscopio (rad²/s²)

% Matriz de covarianza de ruido de medición IMU/Encoders R_imu (9x9)
params.R_imu = diag([...
    0.1^2 * ones(1,3), ...          % Ruido acelerómetro (m²/s⁴)
    0.01^2 * ones(1,3), ...         % Ruido giroscopio (rad²/s²)
    0.05^2 * ones(1,3)]);           % Ruido encoders (m²/s²)

% Matriz de covarianza de ruido de medición cámara R_cam (3x3)
params.R_cam = diag([0.02^2, 0.02^2, 0.02^2]);  % Ruido posición (m²)

%% PARÁMETROS DEL CONTROLADOR SMC (Sliding Mode Control)
params.lambda_smc = 2.5;            % Pendiente de superficie deslizante
params.K_smc = 6.0;                 % Ganancia de conmutación
params.eta_smc = 0.8;               % Ancho de capa límite (anti-chattering)
params.tau_max = 10.0;              % Saturación de torque (Nm)

%% PARÁMETROS DE SENSORES (Para Simulación)
params.noise_enc = 0.05;            % Error de encoders (5%)
params.bias_gyro = [0.01; -0.008; 0.012];  % Bias del giroscopio (rad/s)
params.noise_gyro = 0.005;          % Ruido blanco giroscopio (rad/s)
params.noise_accel = 0.1;           % Ruido acelerómetro (m/s²)
params.noise_cam = 0.02;            % Error de cámara (m)
params.cam_freq = 10;               % Frecuencia de actualización cámara (Hz)
end
