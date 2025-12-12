% test_step_response_closedloop.m
% Prueba de respuesta al escalón en LAZO CERRADO
% Evalúa el sistema CON controlador (Fuzzy + PID) aplicando un step de posición
%
% MÉTRICAS DE DESEMPEÑO:
%   - Tiempo de subida (tr): 10% a 90% del objetivo
%   - Tiempo de asentamiento (ts): ±2% del objetivo
%   - Sobrepaso máximo (Mp): % sobre objetivo
%   - Error en estado estacionario (ess)
%   - IAE: Integral del Error Absoluto
%   - ISE: Integral del Error Cuadrático

clear; clc; close all;

% Agregar rutas
addpath('../config');
addpath('../system');
addpath('../controllers');

fprintf('\n╔════════════════════════════════════════════════════════════╗\n');
fprintf('║   TEST DE RESPUESTA AL ESCALÓN - LAZO CERRADO             ║\n');
fprintf('║   Sistema con Controlador Fuzzy + PID                     ║\n');
fprintf('╚════════════════════════════════════════════════════════════╝\n\n');

%% 1. CARGAR PARÁMETROS
params = robot_parameters();

%% 2. CREAR SISTEMA DE INFERENCIA DIFUSO
fprintf('► Inicializando controlador difuso...\n');
fis = fuzzy_yaw_rate_controller_setup();

%% 3. CONDICIONES INICIALES
% Robot en origen, orientado hacia X positivo
X0 = zeros(12, 1);
X0(5) = deg2rad(0.5); % Pequeño pitch para estabilidad

%% 4. REFERENCIA: ESCALÓN UNITARIO DE POSICIÓN
% El objetivo es mover el robot de X=0 a X=1 metro (escalón unitario)
X_step = 1.0; % metros (escalón unitario de posición)
Y_step = 0.0; % Sin movimiento en Y

target_position = [X_step, Y_step];

fprintf('► Referencia: Escalón de posición (0,0) → (%.1f, %.1f) m\n\n', X_step, Y_step);

%% 5. PARÁMETROS DE SIMULACIÓN
dt_control = 0.01;  % 100 Hz
max_time = 90;      % Tiempo máximo (s)
threshold = 0.05;   % Umbral de llegada (m)

%% 6. SIMULACIÓN EN LAZO CERRADO
fprintf('► Ejecutando simulación en lazo cerrado...\n');

% Almacenamiento
max_steps = max_time / dt_control;
X_history = zeros(max_steps, 12);
U_history = zeros(max_steps, 4);
t_history = zeros(max_steps, 1);
error_history = zeros(max_steps, 1);

X_current = X0;
X_ref = [X_step; Y_step; zeros(10,1)];
X_ref_prev = [0; 0; zeros(10,1)];

step = 1;
t_current = 0;

while step < max_steps && t_current < max_time
    % Guardar estado
    X_history(step,:) = X_current';
    t_history(step) = t_current;
    
    % Calcular error de posición
    error_x = X_step - X_current(1);
    error_y = Y_step - X_current(2);
    error_dist = sqrt(error_x^2 + error_y^2);
    error_history(step) = error_dist;
    
    % Control con Fuzzy + PID

    params.Kp_wheel = 12;
    params.Ki_wheel = 40;
    params.Kd_wheel = 0;

    u_control = fuzzy_yaw_rate_controller(X_current, X_ref, X_ref_prev, ...
        dt_control, fis, params);
    U_history(step,:) = u_control';
    
    % Integrar dinámica
    t_span = [t_current, t_current + dt_control];
    model_func = @(t,x) skid_steer_robot_model(t, x, u_control, params);
    opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
    [~, X_step_sim] = ode45(model_func, t_span, X_current, opts);
    
    % Actualizar
    X_current = X_step_sim(end,:)';
    X_ref_prev = X_ref;
    t_current = t_current + dt_control;
    step = step + 1;
    
    % Progreso cada 20%
    if mod(step, floor(max_steps/5)) == 0
        fprintf('  Progreso: %d%%, Posición: (%.2f, %.2f), Error: %.3f m\n', ...
            round(100*step/max_steps), X_current(1), X_current(2), error_dist);
    end
end

% Recortar arrays
final_step = step - 1;
X_history = X_history(1:final_step, :);
U_history = U_history(1:final_step, :);
t_history = t_history(1:final_step);
error_history = error_history(1:final_step);

fprintf('✓ Simulación completada (%d pasos, %.1f s)\n\n', final_step, t_current);

%% 7. EXTRAER RESPUESTAS
position_x = X_history(:, 1);
position_y = X_history(:, 2);
velocity = X_history(:, 7);
yaw = rad2deg(X_history(:, 6));

%% 8. CALCULAR MÉTRICAS DE DESEMPEÑO

fprintf('══════════════════════════════════════════════════════════════\n');
fprintf('                 MÉTRICAS DE DESEMPEÑO                        \n');
fprintf('══════════════════════════════════════════════════════════════\n\n');

% --- VALOR EN ESTADO ESTACIONARIO ---
n_ss = round(0.1 * length(t_history));
x_ss = mean(position_x(end-n_ss:end));
y_ss = mean(position_y(end-n_ss:end));
error_ss = sqrt((X_step - x_ss)^2 + (Y_step - y_ss)^2);

fprintf('► Estado Estacionario:\n');
fprintf('    Posición final: (%.4f, %.4f) m\n', x_ss, y_ss);
fprintf('    Objetivo: (%.4f, %.4f) m\n', X_step, Y_step);
fprintf('    Error (ess): %.4f m (%.2f%%)\n\n', error_ss, error_ss/X_step*100);

% --- TIEMPO DE SUBIDA (Rise Time) ---
x_10 = 0.1 * X_step;
x_90 = 0.9 * X_step;

idx_10 = find(position_x >= x_10, 1, 'first');
idx_90 = find(position_x >= x_90, 1, 'first');

if ~isempty(idx_10) && ~isempty(idx_90)
    t_10 = t_history(idx_10);
    t_90 = t_history(idx_90);
    rise_time = t_90 - t_10;
else
    t_10 = NaN; t_90 = NaN;
    rise_time = NaN;
end

fprintf('► Tiempo de Subida (Rise Time, tr):\n');
fprintf('    t(10%% = %.2fm): %.4f s\n', x_10, t_10);
fprintf('    t(90%% = %.2fm): %.4f s\n', x_90, t_90);
fprintf('    tr = %.4f s\n\n', rise_time);

% --- TIEMPO DE ASENTAMIENTO (Settling Time) ---
tolerance = 0.02; % 2%
band_upper = X_step * (1 + tolerance);
band_lower = X_step * (1 - tolerance);

settling_time = NaN;
for i = length(t_history):-1:1
    if position_x(i) > band_upper || position_x(i) < band_lower
        if i < length(t_history)
            settling_time = t_history(i+1);
        end
        break;
    end
end

if i == 1 && position_x(1) >= band_lower && position_x(1) <= band_upper
    settling_time = 0;
end

fprintf('► Tiempo de Asentamiento (Settling Time, ts):\n');
fprintf('    Banda: [%.4f, %.4f] m (±%.0f%%)\n', band_lower, band_upper, tolerance*100);
fprintf('    ts = %.4f s\n\n', settling_time);

% --- SOBREPASO MÁXIMO (Overshoot) ---
x_max = max(position_x);
if X_step > 0
    overshoot_pct = max(0, ((x_max - X_step) / X_step) * 100);
else
    overshoot_pct = 0;
end

[~, idx_peak] = max(position_x);
t_peak = t_history(idx_peak);

fprintf('► Sobrepaso Máximo (Overshoot, Mp):\n');
fprintf('    Posición máxima: %.4f m\n', x_max);
fprintf('    Tiempo de pico: %.4f s\n', t_peak);
fprintf('    Mp = %.2f %%\n\n', overshoot_pct);

% --- MÉTRICAS INTEGRALES ---
% IAE: Integral del Error Absoluto
IAE = trapz(t_history, abs(error_history));

% ISE: Integral del Error Cuadrático
ISE = trapz(t_history, error_history.^2);

% ITAE: Integral del Tiempo por Error Absoluto
ITAE = trapz(t_history, t_history .* abs(error_history));

fprintf('► Métricas Integrales:\n');
fprintf('    IAE  (Integral Absolute Error): %.4f m·s\n', IAE);
fprintf('    ISE  (Integral Squared Error):  %.4f m²·s\n', ISE);
fprintf('    ITAE (Integral Time × |Error|): %.4f m·s²\n\n', ITAE);

% --- ESFUERZO DE CONTROL ---
torque_max = max(abs(U_history(:)));
torque_mean = mean(abs(U_history(:)));
energy = trapz(t_history, sum(U_history.^2, 2)); % Energía cuadrática

fprintf('► Esfuerzo de Control:\n');
fprintf('    Torque máximo: %.2f Nm\n', torque_max);
fprintf('    Torque promedio: %.2f Nm\n', torque_mean);
fprintf('    Energía total: %.2f Nm²·s\n\n', energy);

% --- RESUMEN ---
fprintf('══════════════════════════════════════════════════════════════\n');
fprintf('                    RESUMEN DE MÉTRICAS                        \n');
fprintf('══════════════════════════════════════════════════════════════\n');
fprintf('┌─────────────────────────────┬───────────────────────────────┐\n');
fprintf('│ Parámetro                   │ Valor                         │\n');
fprintf('├─────────────────────────────┼───────────────────────────────┤\n');
fprintf('│ Referencia (X_step)         │ %.2f m                        │\n', X_step);
fprintf('│ Posición final (x_ss)       │ %.4f m                       │\n', x_ss);
fprintf('│ Error Estado Est. (ess)     │ %.4f m (%.2f%%)              │\n', error_ss, error_ss/X_step*100);
fprintf('│ Tiempo de Subida (tr)       │ %.4f s                       │\n', rise_time);
fprintf('│ Tiempo de Asentamiento (ts) │ %.4f s                       │\n', settling_time);
fprintf('│ Sobrepaso Máximo (Mp)       │ %.2f %%                       │\n', overshoot_pct);
fprintf('│ IAE                         │ %.4f m·s                     │\n', IAE);
fprintf('│ ISE                         │ %.4f m²·s                    │\n', ISE);
fprintf('└─────────────────────────────┴───────────────────────────────┘\n\n');

%% 9. VISUALIZACIÓN
figure('Name', 'Respuesta al Escalón - Lazo Cerrado', 'Position', [50 50 1400 800]);

% Subplot 1: Posición X (Respuesta Principal)
subplot(2,3,1);
plot(t_history, position_x, 'b-', 'LineWidth', 2);
hold on;
yline(X_step, 'r--', 'LineWidth', 1.5, 'Label', sprintf('Ref = %.1f m', X_step));
fill([0 max(t_history) max(t_history) 0], [band_lower band_lower band_upper band_upper], ...
    'g', 'FaceAlpha', 0.15, 'EdgeColor', 'none');
if ~isnan(settling_time)
    xline(settling_time, 'k--', 'LineWidth', 1, 'Label', sprintf('ts = %.2fs', settling_time));
end
xlabel('Tiempo (s)');
ylabel('Posición X (m)');
title('Respuesta de Posición al Escalón');
legend('x(t)', 'Referencia', 'Location', 'best');
grid on;

% Subplot 2: Error de seguimiento
subplot(2,3,2);
plot(t_history, error_history, 'r-', 'LineWidth', 2);
hold on;
yline(0, 'k--');
xlabel('Tiempo (s)');
ylabel('Error de Posición (m)');
title(sprintf('Error de Seguimiento (IAE = %.3f)', IAE));
grid on;

% Subplot 3: Trayectoria XY
subplot(2,3,3);
plot(position_x, position_y, 'b-', 'LineWidth', 2);
hold on;
plot(0, 0, 'go', 'MarkerSize', 12, 'LineWidth', 2);
plot(X_step, Y_step, 'r*', 'MarkerSize', 15, 'LineWidth', 2);
plot(position_x(end), position_y(end), 'bs', 'MarkerSize', 10, 'LineWidth', 2);
xlabel('X (m)'); ylabel('Y (m)');
title('Trayectoria XY');
legend('Trayectoria', 'Inicio', 'Objetivo', 'Final', 'Location', 'best');
grid on;
axis equal;

% Subplot 4: Torques de control
subplot(2,3,4);
plot(t_history, U_history(:,1), 'r-', 'LineWidth', 1); hold on;
plot(t_history, U_history(:,2), 'b-', 'LineWidth', 1);
plot(t_history, U_history(:,3), 'g--', 'LineWidth', 1);
plot(t_history, U_history(:,4), 'm--', 'LineWidth', 1);
xlabel('Tiempo (s)');
ylabel('Torque (Nm)');
title('Señales de Control');
legend('FL', 'FR', 'RL', 'RR', 'Location', 'best');
grid on;

% Subplot 5: Velocidad
subplot(2,3,5);
plot(t_history, velocity, 'k-', 'LineWidth', 2);
xlabel('Tiempo (s)');
ylabel('Velocidad (m/s)');
title('Velocidad Longitudinal');
grid on;

% Subplot 6: Orientación (Yaw)
subplot(2,3,6);
plot(t_history, yaw, 'm-', 'LineWidth', 2);
xlabel('Tiempo (s)');
ylabel('Yaw (°)');
title('Orientación del Robot');
grid on;

% Título general
sgtitle(sprintf('RESPUESTA AL ESCALÓN - LAZO CERRADO (Ref = %.1f m)', X_step), ...
    'FontSize', 14, 'FontWeight', 'bold');

%% 10. GUARDAR RESULTADOS
results.X_step = X_step;
results.x_ss = x_ss;
results.error_ss = error_ss;
results.rise_time = rise_time;
results.settling_time = settling_time;
results.overshoot_pct = overshoot_pct;
results.IAE = IAE;
results.ISE = ISE;
results.ITAE = ITAE;
results.torque_max = torque_max;
results.energy = energy;
results.t = t_history;
results.position_x = position_x;
results.error = error_history;
results.torques = U_history;

% Guardar en archivo
save('closedloop_step_results.mat', 'results');
fprintf('✓ Resultados guardados en closedloop_step_results.mat\n\n');

%% 11. COMPARACIÓN (si existe archivo de lazo abierto)
if exist('openloop_step_results.mat', 'file')
    fprintf('══════════════════════════════════════════════════════════════\n');
    fprintf('           COMPARACIÓN LAZO ABIERTO vs CERRADO                \n');
    fprintf('══════════════════════════════════════════════════════════════\n');
    
    ol = load('openloop_step_results.mat');
    
    fprintf('┌────────────────────────┬─────────────────┬─────────────────┐\n');
    fprintf('│ Métrica                │ Lazo Abierto    │ Lazo Cerrado    │\n');
    fprintf('├────────────────────────┼─────────────────┼─────────────────┤\n');
    fprintf('│ Tiempo de Subida (tr)  │ %.4f s        │ %.4f s        │\n', ol.results.rise_time, rise_time);
    fprintf('│ Tiempo Asent. (ts)     │ %.4f s        │ %.4f s        │\n', ol.results.settling_time, settling_time);
    fprintf('│ Sobrepaso (Mp)         │ %.2f %%         │ %.2f %%         │\n', ol.results.overshoot_pct, overshoot_pct);
    fprintf('└────────────────────────┴─────────────────┴─────────────────┘\n\n');
end

fprintf('══════════════════════════════════════════════════════════════\n');
fprintf('         TEST DE LAZO CERRADO COMPLETADO                      \n');
fprintf('══════════════════════════════════════════════════════════════\n\n');
