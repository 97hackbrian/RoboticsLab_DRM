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

%% 2. CALCULAR ERRORES DE POSICIÓN Y HEADING
dx = x_ref - x;
dy = y_ref - y;
distance = sqrt(dx^2 + dy^2);

% Ángulo hacia el objetivo
heading_desired = atan2(dy, dx);

% Error de heading (wrap to [-pi, pi])
error_heading = heading_desired - psi;
error_heading = atan2(sin(error_heading), cos(error_heading));

% ═══════════════════════════════════════════════════════════════════════
%  CROSS-TRACK ERROR (Error lateral perpendicular a la trayectoria)
%  Esto detecta cuando el robot "va recto" pero está desviado del camino
% ═══════════════════════════════════════════════════════════════════════
% Proyección del error de posición perpendicular al heading del robot
% cross_track = dx * sin(psi) - dy * cos(psi)
% Positivo = desviado a la derecha, Negativo = desviado a la izquierda
cross_track_error = dx * sin(psi) - dy * cos(psi);

% Añadir cross-track error al error de heading
% Esto FUERZA corrección incluso si el robot "apunta" al objetivo
Kp_crosstrack = 0.8;  % rad/s por metro de desviación lateral
cross_track_omega = Kp_crosstrack * cross_track_error;
cross_track_omega = max(-1.5, min(1.5, cross_track_omega));  % Saturar

% Derivada de error (velocidad de cambio de heading)
persistent prev_error_heading
if isempty(prev_error_heading)
    prev_error_heading = error_heading;
end
derror_heading = (error_heading - prev_error_heading) / max(dt, 0.001);
prev_error_heading = error_heading;

% ═════════════════════════════════════════════════════════════════════
%  COMPENSADOR DE ATRASO (LAG COMPENSATOR) - ANTES DEL FIS
%  C_lag(s) = (s + z) / (s + p), donde z > p
%  Modifica la "distancia percibida" para eliminar error estacionario
% ═════════════════════════════════════════════════════════════════════
persistent x_lag_state
if isempty(x_lag_state)
    x_lag_state = 0;
end

% Parámetros del compensador
alpha_lag = 3.5;        % Factor de mejora del error estacionario
wc_estimate = 0.5;      % Frecuencia de cruce estimada
z_lag = wc_estimate / 10;
p_lag = z_lag / alpha_lag;

% Aplicar compensador a la distancia (entrada del FIS)
% Esto hace que el FIS "perciba" una distancia mayor, evitando que frene demasiado pronto
if distance < 3.0 && distance > 0.08
    % Actualizar estado del compensador
    x_lag_state = (1 - p_lag * dt) * x_lag_state + dt * distance;
    distance_compensation = (z_lag - p_lag) * x_lag_state;
    
    % La distancia compensada es mayor que la real (empuja al robot más cerca)
    distance_compensated = distance + max(0, min(0.5, distance_compensation));
else
    % Muy lejos o muy cerca: resetear y usar distancia real
    x_lag_state = 0;
    distance_compensated = distance;
end

%% 3. EVALUAR FIS (obtener velocidades deseadas)
% Limitar entradas
error_heading_clip = max(-pi, min(pi, error_heading));
derror_heading_clip = max(-2, min(2, derror_heading));
distance_clip = max(0, min(20, distance_compensated));  % Usa distancia COMPENSADA

output = evalfis(fis, [error_heading_clip, derror_heading_clip, distance_clip]);

v_desired = output(1);      % m/s
omega_desired = output(2);  % rad/s

% --- CORRECCIÓN DE ORIENTACIÓN DURANTE MOVIMIENTO ---
% Problema: El FIS solo da omega cuando hay error grande.
% Solución: Agregar corrección proporcional CONTINUA para errores pequeños.

% Ganancia proporcional para corrección de heading durante movimiento
Kp_heading = 20.5;  % rad/s por radián de error (aumentado de 2.0)

% Si hay error de heading pero el FIS dio omega=0 (porque no hay regla activa),
% aplicar corrección proporcional directa
if abs(error_heading) > 0.03 && abs(omega_desired) < 0.1
    omega_correction = Kp_heading * error_heading;
    omega_correction = max(-2.5, min(2.5, omega_correction));  % Saturar a ±2.5 rad/s
    omega_desired = omega_desired + omega_correction;
end

% ═══════════════════════════════════════════════════════════════════════
%  CORRECCIÓN CONTINUA DE CROSS-TRACK ERROR
%  Esto fuerza corrección de deriva lateral MIENTRAS avanza
% ═══════════════════════════════════════════════════════════════════════
% Aplicar cross-track omega SIEMPRE que estemos moviéndonos
% (ya se calculó arriba como cross_track_omega)
if distance > 0.2 && abs(v_desired) > 0.05
    omega_desired = omega_desired + cross_track_omega;
end

% Boost de omega: Si estamos moviéndonos y hay error, aumentar omega
% Esto fuerza la corrección MIENTRAS avanza, no solo parado
if v_desired > 0.1 && abs(error_heading) > 0.08
    % Reducir velocidad proporcionalmente al error de heading
    % (si el heading está muy mal, ir más lento para poder girar)
    heading_factor = max(0.4, 1 - abs(error_heading) / (pi/6));
    v_desired = v_desired * heading_factor;
    
    % Aumentar omega para corregir mientras avanza
    omega_boost = 0.8 * sign(error_heading) * min(abs(error_heading), pi/4);
    omega_desired = omega_desired + omega_boost;
end

% --- REVISIÓN DE PARADA FINAL ---
% IMPORTANTE: Usar distance (real) para la parada final, no la compensada
% El compensador evita frenar demasiado pronto, pero la parada real
% debe basarse en la distancia física verdadera
if distance < 0.01
    omega_desired = 0; % DEADZONE DE GIRO: Prohibido girar muy cerca
    
    if distance < 0.005
        v_desired = 0; % Parada total MUY cerca (5cm)
    else
        v_desired = min(v_desired, 0.1); % Arrastrar muy lento
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
