function torques = fuzzy_position_controller(X, X_ref, X_ref_prev, dt, fis, params)
% fuzzy_position_controller: Controlador difuso para tracking de posición
%
% ENTRADAS:
%   X          - Estado actual [12x1]: [x,y,z,φ,θ,ψ,u,v,w,p,q,r]
%   X_ref      - Estado de referencia [12x1] (principalmente x,y)
%   X_ref_prev - Estado de referencia anterior (para calcular derivada)
%   dt         - Paso de tiempo (s)
%   fis        - Sistema de Inferencia Difuso
%   params     - Parámetros del robot
%
% SALIDA:
%   torques    - Vector de torques [4x1]: [T_FL; T_FR; T_RL; T_RR]

%% 1. EXTRAER ESTADOS
x = X(1); y = X(2);
theta = X(5); % Pitch angle
u_vel = X(7); % Velocidad longitudinal
psi = X(6); % Yaw angle

x_ref = X_ref(1); y_ref = X_ref(2);
x_ref_prev_val = X_ref_prev(1); y_ref_prev_val = X_ref_prev(2);

%% 2. CALCULAR ERRORES EN MARCO GLOBAL
e_x_global = x_ref - x;
e_y_global = y_ref - y;

% Calcular derivada del error (velocidad de seguimiento)
v_ref_x = (x_ref - x_ref_prev_val) / dt;
v_ref_y = (y_ref - y_ref_prev_val) / dt;

% Velocidad actual en marco global (aproximación)
v_x = u_vel * cos(psi);
v_y = u_vel * sin(psi);

de_x = v_ref_x - v_x;
de_y = v_ref_y - v_y;

%% 3. TRANSFORMAR ERRORES AL MARCO DEL ROBOT (opcional - usar global es más simple)
% Para simplificar, usamos errores en marco global
% En un controlador más avanzado, rotaríamos al marco del cuerpo

%% 4. EVALUAR SISTEMA DIFUSO
% Limitar entradas dentro de los rangos del FIS
e_x_clipped = max(-5, min(5, e_x_global));
de_x_clipped = max(-3, min(3, de_x));
e_y_clipped = max(-5, min(5, e_y_global));
pitch_deg = max(-30, min(30, rad2deg(theta)));

% Evaluar FIS
output = evalfis(fis, [e_x_clipped, de_x_clipped, e_y_clipped, pitch_deg]);

T_long = output(1); % Torque longitudinal
T_diff = output(2); % Torque diferencial

%% 5. SATURACIÓN DE TORQUES
tau_max = params.tau_max; % 10 Nm típicamente

T_long = max(-tau_max, min(tau_max, T_long));
T_diff = max(-tau_max/2, min(tau_max/2, T_diff)); % Diferencial con menor límite

%% 6. DISTRIBUIR TORQUES A LAS 4 RUEDAS
% Configuración Skid-Steer:
% - T_long: torque promedio (avance)
% - T_diff: diferencia izquierda-derecha (giro)
%   T_diff > 0 → ruedas derechas más lentas → giro a la derecha
%   T_diff < 0 → ruedas izquierdas más lentas → giro a la izquierda

% Front Left, Front Right, Rear Left, Rear Right
T_left = T_long - T_diff;
T_right = T_long + T_diff;

% Saturar nuevamente después de la distribución
T_left = max(-tau_max, min(tau_max, T_left));
T_right = max(-tau_max, min(tau_max, T_right));

torques = [T_left;   % FL
    T_right;  % FR
    T_left;   % RL
    T_right]; % RR

end
