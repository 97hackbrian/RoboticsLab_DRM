function dX = skid_steer_robot_model(t, X, u, params)
% skid_steer_robot_model: Modelo dinámico de robot 4WD Skid-Steer en terreno 3D
%
% ENTRADAS:
% t      : Tiempo actual (s)
% X      : Vector de Estado [12x1]
%          [x; y; z; phi; theta; psi; u; v; w; p; q; r]
%          Posición (Inercial), Ángulos Euler (Roll, Pitch, Yaw),
%          Velocidades Lineales (Cuerpo), Velocidades Angulares (Cuerpo)
% u      : Entrada de Control [4x1] -> Torques en ruedas [T_FL; T_FR; T_RL; T_RR]
% params : Estructura con parámetros físicos (masa, inercia, geometría)
%
% SALIDA:
% dX     : Derivada del estado (para integrador ode45)

%% 1. DESEMPAQUETAR ESTADO
% Posición y Orientación
% x = X(1); y = X(2); z = X(3); % No se usan para derivadas, solo para registro
phi   = X(4); % Roll
theta = X(5); % Pitch
psi   = X(6); % Yaw

% Velocidades en Marco del Cuerpo (Body Frame)
u_b = X(7); % Surge (Adelante)
v_b = X(8); % Sway (Lateral - deslizamiento)
w_b = X(9); % Heave (Vertical)

p = X(10); % Roll rate
q = X(11); % Pitch rate
r = X(12); % Yaw rate

%% 2. PARÁMETROS FÍSICOS
m = params.m;           % Masa (kg)
I = params.I;           % Matriz de Inercia [3x3]
rw = params.r_wheel;    % Radio rueda
L = params.L;           % Longitud (eje delantero a trasero)
W = params.W;           % Ancho (rueda izq a der)
g = 9.81;

% Parámetros de Incertidumbre / Terreno
mu_s = params.mu_static; % Fricción estática terreno
mu_k = params.mu_kinetic; % Fricción cinética

%% 3. CINEMÁTICA (Relación Cuerpo -> Mundo)
% Matriz de Rotación (Yaw-Pitch-Roll sequence) Body a Inertial (Z-Up)
% Convention:
%   Yaw (psi) around Z, Pitch (theta) around Y, Roll (phi) around X.
%   Positive Pitch (theta > 0) = Nose UP (Climbing) -> Z increases.

cph = cos(phi); sph = sin(phi);
cth = cos(theta); sth = sin(theta); % sth > 0 means Nose UP
cps = cos(psi); sps = sin(psi);

% R_ib derived for Z-Up world, Y-Left body (or standard aerospace with Z inverted)
% R = Rz(psi) * Ry(theta) * Rx(phi)
% Row 1: [cTh*cPs, sPh*sTh*cPs - cPh*sPs, cPh*sTh*cPs + sPh*sPs]
% Row 2: [cTh*sPs, sPh*sTh*sPs + cPh*cPs, cPh*sTh*sPs - sPh*cPs]
% Row 3: [-sTh,    sPh*cTh,               cPh*cTh]  <-- This is for Z-Down!

% FIX: For Z-Up Convention (Nose Up = +Z), we change signs involving sin(theta) in vertical components
% Actually, standard transformation for [X_fwd, Y_left, Z_up]:
R_ib = [cth*cps, sph*sth*cps - cph*sps, cph*sth*cps + sph*sps;
    cth*sps, sph*sth*sps + cph*cps, cph*sth*sps - sph*cps;
    sth,     -sph*cth,              cph*cth];

% Velocidades lineales en mundo
vel_world = R_ib * [u_b; v_b; w_b];

% Tasas de ángulos de Euler (Relación p,q,r -> dphi, dtheta, dpsi)
% Nota: Ojo con la singularidad si theta = +/- 90 grados
T_ang = [1, sph*tan(theta), cph*tan(theta);
    0, cph,            -sph;
    0, sph/cth,        cph/cth];

d_angles = T_ang * [p; q; r];

%% 4. DINÁMICA DE RUEDAS Y DESLIZAMIENTO (SKID STEER)
% Aquí es donde modelamos la "incertidumbre" del terreno.
% Calculamos la velocidad teórica en cada punto de contacto de rueda.

% Geometría de puntos de contacto (Front-Left, Front-Right, etc.)
% Posiciones relativas al centro de masa
d_FL = [ L/2;  W/2; -params.h_chassis];
d_FR = [ L/2; -W/2; -params.h_chassis];
d_RL = [-L/2;  W/2; -params.h_chassis];
d_RR = [-L/2; -W/2; -params.h_chassis];

% Velocidad del suelo en cada rueda (Vel cuerpo + Omega x Radio)
v_wheel_FL = [u_b; v_b; 0] + cross([p;q;r], d_FL);
v_wheel_FR = [u_b; v_b; 0] + cross([p;q;r], d_FR);
v_wheel_RL = [u_b; v_b; 0] + cross([p;q;r], d_RL);
v_wheel_RR = [u_b; v_b; 0] + cross([p;q;r], d_RR);

% Función auxiliar para fuerza de tracción con deslizamiento (Slip)
% u_in: Torque motor, v_long: vel longitudinal rueda, v_lat: vel lateral
% Pasamos angulos (theta, phi) para cálculo correcto de fuerza normal
[F_x_FL, F_y_FL, N_FL] = compute_tire_force(u(1), v_wheel_FL, rw, mu_s, params, theta, phi);
[F_x_FR, F_y_FR, N_FR] = compute_tire_force(u(2), v_wheel_FR, rw, mu_s, params, theta, phi);
[F_x_RL, F_y_RL, N_RL] = compute_tire_force(u(3), v_wheel_RL, rw, mu_s, params, theta, phi);
[F_x_RR, F_y_RR, N_RR] = compute_tire_force(u(4), v_wheel_RR, rw, mu_s, params, theta, phi);

% Fuerzas Totales en el Cuerpo
F_traction_x = F_x_FL + F_x_FR + F_x_RL + F_x_RR;
F_traction_y = F_y_FL + F_y_FR + F_y_RL + F_y_RR; % Resistencia lateral al giro

% CRÍTICO: Fuerza vertical (Normal) total que soporta al robot
% Esta fuerza debe equilibrar la gravedad proyectada + componente Z de aceleración
F_traction_z = N_FL + N_FR + N_RL + N_RR;

% Torques generados por las fuerzas de las ruedas
% INCLUIMOS la componente vertical (normal) para que el suelo genere
% un torque de reacción que evita que el robot se voltee
tau_tires = cross(d_FL, [F_x_FL; F_y_FL; N_FL]) + ...
    cross(d_FR, [F_x_FR; F_y_FR; N_FR]) + ...
    cross(d_RL, [F_x_RL; F_y_RL; N_RL]) + ...
    cross(d_RR, [F_x_RR; F_y_RR; N_RR]);

%% 5. FUERZAS DE GRAVEDAD (PENDIENTES)
% Proyectar gravedad del mundo al cuerpo
F_gravity_body = R_ib' * [0; 0; -m*g];

%% 6. ECUACIONES DE NEWTON-EULER (Dinámica del Cuerpo Rígido)

% Sumatoria de Fuerzas = m * (accel + w x v)
% F_total = F_traction + F_gravity
% AHORA incluimos la componente vertical de las fuerzas de contacto
F_total = [F_traction_x; F_traction_y; F_traction_z] + F_gravity_body;

% Añadir resistencia aerodinámica o fricción viscosa extra si se desea
F_total = F_total - params.Cd * [u_b; v_b; 0] .* abs([u_b; v_b; 0]);

% Derivada velocidades lineales: dot{v} = F/m - w x v
d_vel_b = (F_total / m) - cross([p;q;r], [u_b; v_b; w_b]);

% Sumatoria de Torques = I * dot{w} + w x (I * w)
% tau_total = tau_tires
tau_total = tau_tires;

% Derivada velocidades angulares: dot{w} = inv(I) * (tau - w x (Iw))
d_omega_b = I \ (tau_total - cross([p;q;r], I*[p;q;r]));

%% 7. EMPAQUETAR DERIVADAS
dX = [vel_world; d_angles; d_vel_b; d_omega_b];
end

%% SUB-FUNCIÓN: Modelo de Fricción / Neumático Simplificado
function [Fx, Fy, Fz] = compute_tire_force(Torque, v_contact, r, mu, p, theta, phi)
% Modelo simplificado de tracción longitudinal, fricción lateral y fuerza normal
% v_contact(1) = velocidad longitudinal rueda sobre suelo
% v_contact(2) = velocidad lateral (derrape)
% v_contact(3) = velocidad vertical (penetración/separación del suelo)

% 1. FUERZA NORMAL (Vertical)
% Carga normal ESTIMADA en pendiente (corrección física)
% N = (mg * cos(theta) * cos(phi)) / 4
% Esto reduce la tracción límite en pendientes fuertes
NormalForce = (p.m * 9.81 * cos(theta) * cos(phi)) / 4;

% Evitar valores negativos o cero si el robot vuelca (mas de 90 deg)
NormalForce = max(0, NormalForce);

% La fuerza normal actúa hacia arriba (en marco del cuerpo, +Z)
% Añadimos un amortiguamiento si la rueda se mueve verticalmente
k_damp_vertical = 100; % Amortiguamiento vertical (N·s/m)
Fz = NormalForce - k_damp_vertical * v_contact(3);

% 2. Fuerza Longitudinal (Tracción)
% Relacionamos Torque con Fuerza, pero limitados por fricción estática
F_desired = Torque / r;

% Límite de fricción (Círculo de Kamm)
F_max = mu * NormalForce;

% Modelo de "Slip" suave usando tanh para evitar discontinuidades numéricas
% Si la rueda gira más rápido que el suelo -> fuerza positiva
% Aquí simplificamos asumiendo control de torque directo:
% Saturamos la fuerza que el motor puede aplicar al suelo
if abs(F_desired) > F_max
    Fx = sign(F_desired) * F_max;
else
    Fx = F_desired;
end

% 3. Fuerza Lateral (Resistencia al giro - Skid Steer)
% Esta fuerza se opone a la velocidad lateral v_contact(2)
% Es crucial para que el Skid-Steer gire (debe vencer esta fricción)

% Coeficiente de fricción lateral (suele ser alto)
Fy_max = p.mu_lateral * NormalForce;

% Usamos tanh para suavizar la transición en velocidad cero
k_smooth = 10; % Factor de suavizado
Fy = -Fy_max * tanh(k_smooth * v_contact(2));
end