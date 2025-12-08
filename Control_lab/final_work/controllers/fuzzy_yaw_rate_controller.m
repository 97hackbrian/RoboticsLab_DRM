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

% --- REVISIÓN DE PARADA FINAL ---
% ESTABILIZACIÓN: Problema de "Singularidad"
% Cuando estamos muy cerca (<0.5m), el cálculo de ángulo varía drásticamente.
% SIEMPRE debemos dejar de girar (omega=0) si estamos cerca para evitar "spinning".

if distance < 0.5
    omega_desired = 0; % DEADZONE DE GIRO: Prohibido girar si estamos cerca
    
    % Solo permitimos movimiento lineal final (parking) si estamos alineados
    % o simplemente paramos todo si estamos muy cerca (0.15m)
    if distance < 0.1
        v_desired = 0; % Parada total
    else
        % Entre 0.15m y 0.5m: Acercarse recto y despacio
        v_desired = min(v_desired, 0.2);
    end
end

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

% PASO 3: Control PID en velocidad de rueda
% Intentar leer ganancias desde params (para tuning en tiempo real)
if isfield(params, 'Kp_wheel')
    Kp_wheel = params.Kp_wheel;
    Ki_wheel = params.Ki_wheel;
    Kd_wheel = params.Kd_wheel;
else
    % Valores por defecto (si no están en params)
    Kp_wheel = 15.0;
    Ki_wheel = 7.0;
    Kd_wheel = 0.09;
end

% Errores de velocidad
e_omega_left = omega_wheel_left_desired - omega_wheel_left_actual;
e_omega_right = omega_wheel_right_desired - omega_wheel_right_actual;

% Integral del error (con anti-windup simple)
persistent int_e_omega_left int_e_omega_right
if isempty(int_e_omega_left)
    int_e_omega_left = 0;
    int_e_omega_right = 0;
end

% Acumular integral solo si no estamos saturados (anti-windup básico) o si el error es pequeño
int_e_omega_left = int_e_omega_left + e_omega_left * dt;
int_e_omega_right = int_e_omega_right + e_omega_right * dt;

% Limitar integral para evitar windup excesivo
limit_int = 5.0;
int_e_omega_left = max(-limit_int, min(limit_int, int_e_omega_left));
int_e_omega_right = max(-limit_int, min(limit_int, int_e_omega_right));


% Derivada de error
persistent prev_e_omega_left prev_e_omega_right
if isempty(prev_e_omega_left)
    prev_e_omega_left = 0;
    prev_e_omega_right = 0;
end

de_omega_left = (e_omega_left - prev_e_omega_left) / max(dt, 0.001);
de_omega_right = (e_omega_right - prev_e_omega_right) / max(dt, 0.001);

prev_e_omega_left = e_omega_left;
prev_e_omega_right = e_omega_right;

% PASO 4: Calcular torques con PID
T_left = Kp_wheel * e_omega_left + Ki_wheel * int_e_omega_left + Kd_wheel * de_omega_left;
T_right = Kp_wheel * e_omega_right + Ki_wheel * int_e_omega_right + Kd_wheel * de_omega_right;

% PASO 5: Compensación feedforward (ayuda a acelerar respuesta)
% Torque para vencer inercia rotacional
I_wheel = 0.05; % Aumentado (antes 0.01) por rueda más grande (0.2m)
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
