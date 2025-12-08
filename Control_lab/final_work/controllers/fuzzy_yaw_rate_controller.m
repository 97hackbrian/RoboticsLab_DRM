function torques = fuzzy_yaw_rate_controller(X, X_ref, X_ref_prev, dt, fis, params)
% fuzzy_yaw_rate_controller: Controlador tipo uniciclo con lógica fuzzy
% Controla velocidades (v, omega) y las convierte a torques de skid-steer
%
% ENTRADAS:
%   X          - Estado actual [12x1]
%   X_ref      - Estado de referencia [12x1]
%   X_ref_prev - Estado anterior (para derivada)
%   dt         - Paso de tiempo
%   fis        - FIS de velocidades
%   params     - Parámetros del robot
%
% SALIDA:
%   torques - [4x1] torques para ruedas [FL; FR; RL; RR]

%% 1. EXTRAER ESTADOS
x = X(1); y = X(2);
psi = X(6); % Yaw actual
u_vel = X(7); % Velocidad actual

x_ref = X_ref(1); y_ref = X_ref(2);
x_ref_prev_val = X_ref_prev(1);
y_ref_prev_val = X_ref_prev(2);

%% 2. CALCULAR ERROR DE HEADING (como PID uniciclo)
dx = x_ref - x;
dy = y_ref - y;
distance = sqrt(dx^2 + dy^2);

% Ángulo hacia el objetivo
heading_desired = atan2(dy, dx);

% Error de heading (wrap to [-pi, pi])
error_heading = heading_desired - psi;
error_heading = atan2(sin(error_heading), cos(error_heading));

% Derivada de error (velocidad de cambio de heading)
% Aproximación: cambio de error / dt
persistent prev_error_heading
if isempty(prev_error_heading)
    prev_error_heading = error_heading;
end
derror_heading = (error_heading - prev_error_heading) / max(dt, 0.001);
prev_error_heading = error_heading;

%% 3. EVALUAR FIS (obtener velocidades deseadas)
% Limitar entradas
error_heading_clip = max(-pi, min(pi, error_heading));
derror_heading_clip = max(-2, min(2, derror_heading));
distance_clip = max(0, min(20, distance));

output = evalfis(fis, [error_heading_clip, derror_heading_clip, distance_clip]);

v_desired = output(1);      % m/s
omega_desired = output(2);  % rad/s

%% 4. CONVERTIR VELOCIDADES DESEADAS A TORQUES (MEJORADO)
% Modelo realista de skid-steer con control de retroalimentación

W = params.W; % ancho del robot (m)
r = params.r_wheel; % radio de rueda (m)
m = params.m; % masa del robot (kg)

% Extraer velocidades actuales del estado
vx_actual = X(7); % Velocidad en x (body frame)
vy_actual = X(8); % Velocidad en y
omega_actual = X(12); % Velocidad angular actual (rad/s)

% Velocidad lineal actual (longitudinal en body frame)
% IMPORTANTE: Usar velocidad con signo para manejar retrocesos
v_actual = vx_actual;

% PASO 1: Calcular velocidades de ruedas deseadas
% Para skid-steer: v_left = v - omega*W/2, v_right = v + omega*W/2
v_left_desired = v_desired - (omega_desired * W / 2);
v_right_desired = v_desired + (omega_desired * W / 2);

% Convertir a velocidades angulares de ruedas (rad/s)
omega_wheel_left_desired = v_left_desired / r;
omega_wheel_right_desired = v_right_desired / r;

% PASO 2: Estimar velocidades actuales de ruedas desde estado del robot
% Aproximación: desde velocidad del cuerpo y omega
v_left_actual = v_actual - (omega_actual * W / 2);
v_right_actual = v_actual + (omega_actual * W / 2);
omega_wheel_left_actual = v_left_actual / r;
omega_wheel_right_actual = v_right_actual / r;

% PASO 3: Control PD en velocidad de rueda (más suave para evitar chattering)
Kp_wheel = 8.0;  % Reducido de 15.0 para suavizar respuesta
Kd_wheel = 0.5;  % Reducido de 2.0 para reducir ruido/vibración

% Errores de velocidad
e_omega_left = omega_wheel_left_desired - omega_wheel_left_actual;
e_omega_right = omega_wheel_right_desired - omega_wheel_right_actual;

% Derivada de error (aproximación con diferencia finita)
persistent prev_e_omega_left prev_e_omega_right
if isempty(prev_e_omega_left)
    prev_e_omega_left = 0;
    prev_e_omega_right = 0;
end

de_omega_left = (e_omega_left - prev_e_omega_left) / max(dt, 0.001);
de_omega_right = (e_omega_right - prev_e_omega_right) / max(dt, 0.001);

prev_e_omega_left = e_omega_left;
prev_e_omega_right = e_omega_right;

% PASO 4: Calcular torques con PD
T_left = Kp_wheel * e_omega_left + Kd_wheel * de_omega_left;
T_right = Kp_wheel * e_omega_right + Kd_wheel * de_omega_right;

% PASO 5: Compensación feedforward (ayuda a acelerar respuesta)
% Torque para vencer inercia rotacional
I_wheel = 0.01; % kg*m^2 (inercia aproximada de rueda)
T_ff_left = I_wheel * (omega_wheel_left_desired - omega_wheel_left_actual) / max(dt, 0.001);
T_ff_right = I_wheel * (omega_wheel_right_desired - omega_wheel_right_actual) / max(dt, 0.001);

% Limitar feedforward
T_ff_left = max(-3, min(3, T_ff_left));
T_ff_right = max(-3, min(3, T_ff_right));

% Combinar feedback + feedforward
T_left = T_left + 0.3 * T_ff_left;
T_right = T_right + 0.3 * T_ff_right;

%% 5. SATURACIÓN
tau_max = params.tau_max;
T_left = max(-tau_max, min(tau_max, T_left));
T_right = max(-tau_max, min(tau_max, T_right));

%% 6. DISTRIBUIR A LAS 4 RUEDAS
torques = [T_left;   % FL
    T_right;  % FR
    T_left;   % RL
    T_right]; % RR

end
