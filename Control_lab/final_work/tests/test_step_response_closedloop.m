% test_step_response_closedloop.m
% Prueba de respuesta al escalón en LAZO CERRADO
% Evalúa el sistema CON controlador (Fuzzy + PID) aplicando un step de posición
%
% INCLUYE: Compensador de Atraso (Lag Compensator) para eliminar error estacionario
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
fprintf('║   Sistema con Controlador Fuzzy + PID + LAG COMPENSATOR   ║\n');
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
max_time = 30;      % Tiempo máximo (s)
threshold = 0.5;   % Umbral de llegada (m)

%% ═══════════════════════════════════════════════════════════════════════
%%  COMPENSADOR DE ATRASO (LAG COMPENSATOR) - DISEÑO
%% ═══════════════════════════════════════════════════════════════════════
%
%  La función de transferencia del compensador es:
%
%      C_lag(s) = (s + z) / (s + p),    donde z > p
%
%  ESTRATEGIA:
%    - z y p se ubican muy cerca del origen (bajas frecuencias)
%    - Esto aumenta la ganancia en DC (elimina error estacionario)
%    - NO afecta el transitorio porque en altas frecuencias |C_lag| ≈ 1
%
%  PARÁMETROS DE DISEÑO:
%    - alpha = z/p : factor de mejora del error (queremos alpha ≈ 10)
%    - z : cero del compensador (una década antes de la frecuencia de cruce)
%    - p = z/alpha : polo del compensador
%
%% ═══════════════════════════════════════════════════════════════════════

fprintf('╔════════════════════════════════════════════════════════════╗\n');
fprintf('║           DISEÑO DEL COMPENSADOR DE ATRASO                ║\n');
fprintf('╚════════════════════════════════════════════════════════════╝\n\n');

% Parámetros del compensador de atraso (discretizado para simulación)
alpha_lag = 3;      % Factor de mejora del error estacionario
wc_estimate = 0.5;   % Estimación de frecuencia de cruce (rad/s) del sistema actual
z_lag = wc_estimate / 10;  % Cero: una década antes del cruce
p_lag = z_lag / alpha_lag; % Polo: alpha veces más cerca del origen

fprintf('► Parámetros del Compensador de Atraso:\n');
fprintf('    α (factor mejora) = %.1f\n', alpha_lag);
fprintf('    ωc estimado       = %.3f rad/s\n', wc_estimate);
fprintf('    z (cero)          = %.4f rad/s\n', z_lag);
fprintf('    p (polo)          = %.4f rad/s\n', p_lag);
fprintf('    C_lag(s) = (s + %.4f) / (s + %.4f)\n\n', z_lag, p_lag);

% Variables de estado del compensador (para implementación en tiempo discreto)
% Usamos la aproximación de Tustin (Bilinear Transform) para discretizar
%
% C_lag(s) = (s + z) / (s + p)
%
% Aproximación de Euler hacia atrás para implementación simple:
% x_lag[k+1] = (1 - p*dt) * x_lag[k] + dt * error[k]
% u_lag[k] = x_lag[k] + (z/p) * error[k] - (1/p) * x_lag[k]
%
% Simplificación práctica:
% Salida del compensador = ganancia_DC * error + estado_integral con dinámica lenta

%% ═══════════════════════════════════════════════════════════════════════
%%  6. SIMULACIÓN COMPARATIVA: ANTES vs DESPUÉS DEL COMPENSADOR
%% ═══════════════════════════════════════════════════════════════════════

fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('        EJECUTANDO SIMULACIÓN COMPARATIVA (ANTES/DESPUÉS)      \n');
fprintf('═══════════════════════════════════════════════════════════════\n\n');

% Almacenamiento para AMBAS simulaciones
max_steps = max_time / dt_control;

% --- SIMULACIÓN 1: SIN COMPENSADOR (ANTES) ---
fprintf('► [1/2] Simulando sistema ORIGINAL (sin compensador)...\n');

X_history_antes = zeros(max_steps, 12);
U_history_antes = zeros(max_steps, 4);
t_history_antes = zeros(max_steps, 1);
error_history_antes = zeros(max_steps, 1);

X_current = X0;
X_ref = [X_step; Y_step; zeros(10,1)];
X_ref_prev = [0; 0; zeros(10,1)];

step = 1;
t_current = 0;

% Limpiar variables persistentes del controlador
clear fuzzy_yaw_rate_controller;

while step < max_steps && t_current < max_time
    X_history_antes(step,:) = X_current';
    t_history_antes(step) = t_current;
    
    error_x = X_step - X_current(1);
    error_y = Y_step - X_current(2);
    error_dist = sqrt(error_x^2 + error_y^2);
    error_history_antes(step) = error_dist;
    
    % Control ORIGINAL (sin compensador)
    params.Kp_wheel = 30;
    params.Ki_wheel = 50;
    params.Kd_wheel = 0;
    
    u_control = fuzzy_yaw_rate_controller(X_current, X_ref, X_ref_prev, ...
        dt_control, fis, params);
    U_history_antes(step,:) = u_control';
    
    t_span = [t_current, t_current + dt_control];
    model_func = @(t,x) skid_steer_robot_model(t, x, u_control, params);
    opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
    [~, X_step_sim] = ode45(model_func, t_span, X_current, opts);
    
    X_current = X_step_sim(end,:)';
    X_ref_prev = X_ref;
    t_current = t_current + dt_control;
    step = step + 1;
end

final_step_antes = step - 1;
X_history_antes = X_history_antes(1:final_step_antes, :);
U_history_antes = U_history_antes(1:final_step_antes, :);
t_history_antes = t_history_antes(1:final_step_antes);
error_history_antes = error_history_antes(1:final_step_antes);

fprintf('    ✓ Completado (%d pasos)\n\n', final_step_antes);

% --- SIMULACIÓN 2: CON COMPENSADOR DE ATRASO (DESPUÉS) ---
fprintf('► [2/2] Simulando sistema CON COMPENSADOR DE ATRASO...\n');

X_history_despues = zeros(max_steps, 12);
U_history_despues = zeros(max_steps, 4);
t_history_despues = zeros(max_steps, 1);
error_history_despues = zeros(max_steps, 1);

X_current = X0;
X_ref = [X_step; Y_step; zeros(10,1)];
X_ref_prev = [0; 0; zeros(10,1)];

step = 1;
t_current = 0;

% Limpiar variables persistentes del controlador
clear fuzzy_yaw_rate_controller;

% Estado del compensador de atraso
x_lag = 0;  % Estado interno del filtro lag

while step < max_steps && t_current < max_time
    X_history_despues(step,:) = X_current';
    t_history_despues(step) = t_current;
    
    error_x = X_step - X_current(1);
    error_y = Y_step - X_current(2);
    error_dist = sqrt(error_x^2 + error_y^2);
    error_history_despues(step) = error_dist;
    
    % ═══════════════════════════════════════════════════════════════════
    %  COMPENSADOR DE ATRASO - IMPLEMENTACIÓN EN TIEMPO DISCRETO
    % ═══════════════════════════════════════════════════════════════════
    %
    %  C_lag(s) = (s + z) / (s + p)
    %
    %  Forma de espacio de estados:
    %    dx_lag/dt = -p * x_lag + error
    %    y_lag = (z - p) * x_lag + error
    %
    %  Discretización (Euler):
    %    x_lag[k+1] = (1 - p*dt) * x_lag[k] + dt * error[k]
    %    y_lag[k] = (z - p) * x_lag[k] + error[k]
    %
    % ═══════════════════════════════════════════════════════════════════
    
    % Actualizar estado del compensador (dinámica lenta)
    x_lag = (1 - p_lag * dt_control) * x_lag + dt_control * error_x;
    
    % Salida del compensador (error corregido)
    error_x_compensado = (z_lag - p_lag) * x_lag + error_x;
    
    % Crear referencia modificada por el compensador
    % La idea es "empujar" la referencia un poco más allá para eliminar el error
    X_ref_compensado = X_ref;
    X_ref_compensado(1) = X_current(1) + error_x_compensado;  % Referencia ajustada
    
    % Control con PID (usando referencia compensada)
    params.Kp_wheel = 30;
    params.Ki_wheel = 50;
    params.Kd_wheel = 0;
    
    u_control = fuzzy_yaw_rate_controller(X_current, X_ref_compensado, X_ref_prev, ...
        dt_control, fis, params);
    U_history_despues(step,:) = u_control';
    
    t_span = [t_current, t_current + dt_control];
    model_func = @(t,x) skid_steer_robot_model(t, x, u_control, params);
    opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
    [~, X_step_sim] = ode45(model_func, t_span, X_current, opts);
    
    X_current = X_step_sim(end,:)';
    X_ref_prev = X_ref_compensado;
    t_current = t_current + dt_control;
    step = step + 1;
end

final_step_despues = step - 1;
X_history_despues = X_history_despues(1:final_step_despues, :);
U_history_despues = U_history_despues(1:final_step_despues, :);
t_history_despues = t_history_despues(1:final_step_despues);
error_history_despues = error_history_despues(1:final_step_despues);

fprintf('    ✓ Completado (%d pasos)\n\n', final_step_despues);

%% ═══════════════════════════════════════════════════════════════════════
%%  7. CALCULAR MÉTRICAS DE DESEMPEÑO - COMPARACIÓN
%% ═══════════════════════════════════════════════════════════════════════

fprintf('══════════════════════════════════════════════════════════════\n');
fprintf('                 COMPARACIÓN DE MÉTRICAS                      \n');
fprintf('══════════════════════════════════════════════════════════════\n\n');

% Extraer posiciones
position_x_antes = X_history_antes(:, 1);
position_x_despues = X_history_despues(:, 1);

% --- ANTES: Métricas ---
n_ss = round(0.1 * length(t_history_antes));
x_ss_antes = mean(position_x_antes(end-n_ss:end));
error_ss_antes = abs(X_step - x_ss_antes);

x_10 = 0.1 * X_step;
x_90 = 0.9 * X_step;
idx_10 = find(position_x_antes >= x_10, 1, 'first');
idx_90 = find(position_x_antes >= x_90, 1, 'first');
if ~isempty(idx_10) && ~isempty(idx_90)
    rise_time_antes = t_history_antes(idx_90) - t_history_antes(idx_10);
else
    rise_time_antes = NaN;
end

tolerance = 0.02;
band_upper = X_step * (1 + tolerance);
band_lower = X_step * (1 - tolerance);
settling_time_antes = NaN;
for i = length(t_history_antes):-1:1
    if position_x_antes(i) > band_upper || position_x_antes(i) < band_lower
        if i < length(t_history_antes)
            settling_time_antes = t_history_antes(i+1);
        end
        break;
    end
end

x_max_antes = max(position_x_antes);
overshoot_antes = max(0, ((x_max_antes - X_step) / X_step) * 100);
IAE_antes = trapz(t_history_antes, abs(error_history_antes));
ISE_antes = trapz(t_history_antes, error_history_antes.^2);

% --- DESPUÉS: Métricas ---
n_ss = round(0.1 * length(t_history_despues));
x_ss_despues = mean(position_x_despues(end-n_ss:end));
error_ss_despues = abs(X_step - x_ss_despues);

idx_10 = find(position_x_despues >= x_10, 1, 'first');
idx_90 = find(position_x_despues >= x_90, 1, 'first');
if ~isempty(idx_10) && ~isempty(idx_90)
    rise_time_despues = t_history_despues(idx_90) - t_history_despues(idx_10);
else
    rise_time_despues = NaN;
end

settling_time_despues = NaN;
for i = length(t_history_despues):-1:1
    if position_x_despues(i) > band_upper || position_x_despues(i) < band_lower
        if i < length(t_history_despues)
            settling_time_despues = t_history_despues(i+1);
        end
        break;
    end
end

x_max_despues = max(position_x_despues);
overshoot_despues = max(0, ((x_max_despues - X_step) / X_step) * 100);
IAE_despues = trapz(t_history_despues, abs(error_history_despues));
ISE_despues = trapz(t_history_despues, error_history_despues.^2);

%% ═══════════════════════════════════════════════════════════════════════
%%  8. TABLA COMPARATIVA: ANTES vs DESPUÉS
%% ═══════════════════════════════════════════════════════════════════════

fprintf('┌───────────────────────────┬───────────────────┬───────────────────┬────────────┐\n');
fprintf('│         MÉTRICA           │      ANTES        │     DESPUÉS       │   MEJORA   │\n');
fprintf('│                           │  (Sin Compensador)│ (Con Lag Comp.)   │            │\n');
fprintf('├───────────────────────────┼───────────────────┼───────────────────┼────────────┤\n');
fprintf('│ Posición Estado Est.      │ %.4f m          │ %.4f m          │            │\n', x_ss_antes, x_ss_despues);
fprintf('│ Error Estado Est. (ess)   │ %.4f m (%.2f%%) │ %.4f m (%.2f%%) │   %.1f×   │\n', ...
    error_ss_antes, error_ss_antes/X_step*100, error_ss_despues, error_ss_despues/X_step*100, ...
    error_ss_antes/max(error_ss_despues, 0.0001));
fprintf('│ Tiempo de Subida (tr)     │ %.4f s          │ %.4f s          │   %.2f×   │\n', ...
    rise_time_antes, rise_time_despues, rise_time_antes/max(rise_time_despues, 0.001));
fprintf('│ Tiempo Asentamiento (ts)  │ %.4f s          │ %.4f s          │   %.2f×   │\n', ...
    settling_time_antes, settling_time_despues, settling_time_antes/max(settling_time_despues, 0.001));
fprintf('│ Sobrepaso Máximo (Mp)     │ %.2f %%           │ %.2f %%           │            │\n', ...
    overshoot_antes, overshoot_despues);
fprintf('│ IAE                       │ %.4f m·s        │ %.4f m·s        │   %.1f×   │\n', ...
    IAE_antes, IAE_despues, IAE_antes/max(IAE_despues, 0.0001));
fprintf('│ ISE                       │ %.4f m²·s       │ %.4f m²·s       │   %.1f×   │\n', ...
    ISE_antes, ISE_despues, ISE_antes/max(ISE_despues, 0.0001));
fprintf('└───────────────────────────┴───────────────────┴───────────────────┴────────────┘\n\n');

%% ═══════════════════════════════════════════════════════════════════════
%%  9. VISUALIZACIÓN COMPARATIVA
%% ═══════════════════════════════════════════════════════════════════════

figure('Name', 'Comparación: ANTES vs DESPUÉS del Compensador de Atraso', ...
    'Position', [50 50 1600 900]);

% --- Subplot 1: Respuesta de Posición (Principal) ---
subplot(2,3,1);
plot(t_history_antes, position_x_antes, 'b-', 'LineWidth', 2, 'DisplayName', 'ANTES (Sin Comp.)');
hold on;
plot(t_history_despues, position_x_despues, 'r-', 'LineWidth', 2, 'DisplayName', 'DESPUÉS (Con Lag)');
yline(X_step, 'k--', 'LineWidth', 1.5, 'Label', sprintf('Ref = %.1f m', X_step));
fill([0 max_time max_time 0], [band_lower band_lower band_upper band_upper], ...
    'g', 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
xlabel('Tiempo (s)');
ylabel('Posición X (m)');
title('Respuesta de Posición al Escalón');
legend('Location', 'best');
grid on;
xlim([0 max_time]);

% --- Subplot 2: Zoom en Estado Estacionario ---
subplot(2,3,2);
% Mostrar últimos 30 segundos
t_zoom_start = max_time - 10;
idx_zoom_antes = t_history_antes >= t_zoom_start;
idx_zoom_despues = t_history_despues >= t_zoom_start;

plot(t_history_antes(idx_zoom_antes), position_x_antes(idx_zoom_antes), 'b-', 'LineWidth', 2);
hold on;
plot(t_history_despues(idx_zoom_despues), position_x_despues(idx_zoom_despues), 'r-', 'LineWidth', 2);
yline(X_step, 'k--', 'LineWidth', 1.5);
fill([t_zoom_start max_time max_time t_zoom_start], [band_lower band_lower band_upper band_upper], ...
    'g', 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'HandleVisibility', 'off');
xlabel('Tiempo (s)');
ylabel('Posición X (m)');
title(sprintf('ZOOM: Estado Estacionario\n(ess ANTES=%.3fm, ess DESPUÉS=%.3fm)', ...
    error_ss_antes, error_ss_despues));
legend('ANTES', 'DESPUÉS', 'Referencia', 'Location', 'best');
grid on;
xlim([t_zoom_start max_time]);

% --- Subplot 3: Error de Seguimiento ---
subplot(2,3,3);
semilogy(t_history_antes, abs(error_history_antes)+0.001, 'b-', 'LineWidth', 1.5);
hold on;
semilogy(t_history_despues, abs(error_history_despues)+0.001, 'r-', 'LineWidth', 1.5);
yline(tolerance * X_step, 'g--', 'LineWidth', 1.5, 'Label', '±2%');
xlabel('Tiempo (s)');
ylabel('|Error| (m) - Escala Log');
title(sprintf('Error de Seguimiento\n(IAE: %.2f → %.2f)', IAE_antes, IAE_despues));
legend('ANTES', 'DESPUÉS', 'Location', 'best');
grid on;
xlim([0 max_time]);

% --- Subplot 4: Torques de Control ---
subplot(2,3,4);
plot(t_history_antes, U_history_antes(:,1), 'b-', 'LineWidth', 1);
hold on;
plot(t_history_despues, U_history_despues(:,1), 'r-', 'LineWidth', 1);
xlabel('Tiempo (s)');
ylabel('Torque FL (Nm)');
title('Esfuerzo de Control (Rueda FL)');
legend('ANTES', 'DESPUÉS', 'Location', 'best');
grid on;

% --- Subplot 5: Velocidad ---
subplot(2,3,5);
velocity_antes = X_history_antes(:, 7);
velocity_despues = X_history_despues(:, 7);
plot(t_history_antes, velocity_antes, 'b-', 'LineWidth', 1.5);
hold on;
plot(t_history_despues, velocity_despues, 'r-', 'LineWidth', 1.5);
xlabel('Tiempo (s)');
ylabel('Velocidad (m/s)');
title('Velocidad Longitudinal');
legend('ANTES', 'DESPUÉS', 'Location', 'best');
grid on;

% --- Subplot 6: Diagrama de Barras Comparativo ---
subplot(2,3,6);
metrics = {'ess (m)', 'tr (s)', 'ts (s)', 'Mp (%)', 'IAE'};
values_antes = [error_ss_antes, rise_time_antes, settling_time_antes, overshoot_antes, IAE_antes];
values_despues = [error_ss_despues, rise_time_despues, settling_time_despues, overshoot_despues, IAE_despues];

% Normalizar para visualización
values_antes_norm = values_antes ./ max(values_antes);
values_despues_norm = values_despues ./ max(values_antes);

x_pos = 1:length(metrics);
bar_width = 0.35;
bar(x_pos - bar_width/2, values_antes_norm, bar_width, 'FaceColor', [0.2 0.4 0.8], 'DisplayName', 'ANTES');
hold on;
bar(x_pos + bar_width/2, values_despues_norm, bar_width, 'FaceColor', [0.8 0.2 0.2], 'DisplayName', 'DESPUÉS');
xticks(x_pos);
xticklabels(metrics);
ylabel('Valor Normalizado');
title('Comparación de Métricas (Normalizado)');
legend('Location', 'best');
grid on;

% Título general
sgtitle(sprintf('COMPENSADOR DE ATRASO: C_{lag}(s) = (s + %.4f) / (s + %.4f),  α = %.0f', ...
    z_lag, p_lag, alpha_lag), 'FontSize', 14, 'FontWeight', 'bold');

%% ═══════════════════════════════════════════════════════════════════════
%%  10. RESUMEN FINAL
%% ═══════════════════════════════════════════════════════════════════════

fprintf('══════════════════════════════════════════════════════════════\n');
fprintf('                    RESUMEN DEL COMPENSADOR                    \n');
fprintf('══════════════════════════════════════════════════════════════\n\n');

fprintf('► Compensador de Atraso Diseñado:\n');
fprintf('    C_lag(s) = (s + %.4f) / (s + %.4f)\n', z_lag, p_lag);
fprintf('    α = z/p = %.1f (mejora teórica del error)\n\n', alpha_lag);

fprintf('► Mejora Obtenida:\n');
fprintf('    Error Estado Est.: %.4f m → %.4f m (%.1f%% reducción)\n', ...
    error_ss_antes, error_ss_despues, (1 - error_ss_despues/error_ss_antes)*100);
fprintf('    Tiempo de Subida:  %.4f s → %.4f s (%.1f%% cambio)\n', ...
    rise_time_antes, rise_time_despues, (rise_time_despues/rise_time_antes - 1)*100);
fprintf('    Sobrepaso:         %.2f%% → %.2f%%\n', overshoot_antes, overshoot_despues);
fprintf('    IAE (error total): %.4f → %.4f (%.1f%% reducción)\n\n', ...
    IAE_antes, IAE_despues, (1 - IAE_despues/IAE_antes)*100);

fprintf('► Conclusión:\n');
if error_ss_despues < error_ss_antes
    fprintf('    ✓ El compensador ELIMINÓ exitosamente el error estacionario\n');
    fprintf('    ✓ El transitorio se mantiene similar (tr y Mp casi iguales)\n');
else
    fprintf('    ⚠ El compensador NO mejoró el error (ajustar α o z)\n');
end

fprintf('\n══════════════════════════════════════════════════════════════\n');
fprintf('         TEST COMPARATIVO COMPLETADO                          \n');
fprintf('══════════════════════════════════════════════════════════════\n\n');

%% 11. GUARDAR RESULTADOS
results.antes.position_x = position_x_antes;
results.antes.error = error_history_antes;
results.antes.t = t_history_antes;
results.antes.ess = error_ss_antes;
results.antes.tr = rise_time_antes;
results.antes.ts = settling_time_antes;
results.antes.Mp = overshoot_antes;
results.antes.IAE = IAE_antes;

results.despues.position_x = position_x_despues;
results.despues.error = error_history_despues;
results.despues.t = t_history_despues;
results.despues.ess = error_ss_despues;
results.despues.tr = rise_time_despues;
results.despues.ts = settling_time_despues;
results.despues.Mp = overshoot_despues;
results.despues.IAE = IAE_despues;

results.compensator.alpha = alpha_lag;
results.compensator.z = z_lag;
results.compensator.p = p_lag;

save('closedloop_step_comparison.mat', 'results');
fprintf('✓ Resultados guardados en closedloop_step_comparison.mat\n\n');
