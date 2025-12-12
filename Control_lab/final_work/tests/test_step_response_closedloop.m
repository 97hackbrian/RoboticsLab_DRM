% test_step_response_closedloop.m
% Prueba COMPARATIVA de respuesta al escalón en LAZO CERRADO
% Compara 3 configuraciones de control:
%   1. Solo Fuzzy (sin PID, sin Lag Compensator)
%   2. Fuzzy + PID (sin Lag Compensator)
%   3. Fuzzy + PID + Lag Compensator (implementación completa)
%
% MÉTRICAS DE DESEMPEÑO:
%   - Tiempo de subida (tr): 10% a 90% del objetivo
%   - Tiempo de asentamiento (ts): ±2% del objetivo
%   - Sobrepaso máximo (Mp): % sobre objetivo
%   - Error en estado estacionario (ess)
%   - IAE: Integral del Error Absoluto

clear; clc; close all;

% Agregar rutas
addpath('../config');
addpath('../system');
addpath('../controllers');

fprintf('\n╔════════════════════════════════════════════════════════════╗\n');
fprintf('║   TEST COMPARATIVO DE RESPUESTA AL ESCALÓN                ║\n');
fprintf('║   Comparación: Solo Fuzzy vs Fuzzy+PID vs Completo        ║\n');
fprintf('╚════════════════════════════════════════════════════════════╝\n\n');

%% 1. CARGAR PARÁMETROS BASE
params_base = robot_parameters();

%% 2. CREAR SISTEMA DE INFERENCIA DIFUSO
fprintf('► Inicializando controlador difuso...\n');
fis = fuzzy_yaw_rate_controller_setup();

%% 3. CONDICIONES INICIALES
X0 = zeros(12, 1);
X0(5) = deg2rad(0.5);

%% 4. REFERENCIA
X_step = 1.0; % metros
Y_step = 0.0;
fprintf('► Referencia: (0,0) → (%.1f, %.1f) m\n\n', X_step, Y_step);

%% 5. PARÁMETROS DE SIMULACIÓN
dt_control = 0.01;
max_time = 25;
max_steps = max_time / dt_control;

%% ═══════════════════════════════════════════════════════════════════════
%%  6. EJECUTAR 3 SIMULACIONES COMPARATIVAS
%% ═══════════════════════════════════════════════════════════════════════

% Definir configuraciones de control
configs = {
    'Solo Fuzzy',        0,  0,  0, false;  % Kp, Ki, Kd, use_lag
    'Fuzzy + PID',      30, 50,  0, false;
    'Fuzzy + PID + Lag',30, 50,  0, true;
    };

% Colores para gráficos
colors = {'b', 'g', 'r'};
line_styles = {'--', '-.', '-'};

% Almacenamiento de resultados
all_results = cell(3, 1);

for cfg = 1:3
    config_name = configs{cfg, 1};
    Kp = configs{cfg, 2};
    Ki = configs{cfg, 3};
    Kd = configs{cfg, 4};
    use_lag = configs{cfg, 5};
    
    fprintf('═══════════════════════════════════════════════════════════════\n');
    fprintf('► [%d/3] Simulando: %s\n', cfg, config_name);
    fprintf('    Kp=%.0f, Ki=%.0f, Kd=%.0f, Lag=%s\n', Kp, Ki, Kd, string(use_lag));
    fprintf('═══════════════════════════════════════════════════════════════\n');
    
    % Reiniciar arrays
    X_history = zeros(max_steps, 12);
    U_history = zeros(max_steps, 4);
    t_history = zeros(max_steps, 1);
    error_history = zeros(max_steps, 1);
    
    X_current = X0;
    X_ref = [X_step; Y_step; zeros(10,1)];
    X_ref_prev = [0; 0; zeros(10,1)];
    
    step = 1;
    t_current = 0;
    
    % Limpiar variables persistentes del controlador
    clear fuzzy_yaw_rate_controller;
    
    % Copiar parámetros base
    params = params_base;
    params.Kp_wheel = Kp;
    params.Ki_wheel = Ki;
    params.Kd_wheel = Kd;
    params.use_lag_compensator = use_lag;  % Flag para el controlador
    
    while step < max_steps && t_current < max_time
        X_history(step,:) = X_current';
        t_history(step) = t_current;
        
        error_x = X_step - X_current(1);
        error_y = Y_step - X_current(2);
        error_dist = sqrt(error_x^2 + error_y^2);
        error_history(step) = error_dist;
        
        % Calcular control
        u_control = fuzzy_yaw_rate_controller(X_current, X_ref, X_ref_prev, ...
            dt_control, fis, params);
        U_history(step,:) = u_control';
        
        % Integrar dinámica
        t_span = [t_current, t_current + dt_control];
        model_func = @(t,x) skid_steer_robot_model(t, x, u_control, params);
        opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
        [~, X_step_sim] = ode45(model_func, t_span, X_current, opts);
        
        X_current = X_step_sim(end,:)';
        X_ref_prev = X_ref;
        t_current = t_current + dt_control;
        step = step + 1;
    end
    
    % Recortar arrays
    final_step = step - 1;
    X_history = X_history(1:final_step, :);
    U_history = U_history(1:final_step, :);
    t_history = t_history(1:final_step);
    error_history = error_history(1:final_step);
    
    % Extraer respuestas
    position_x = X_history(:, 1);
    position_y = X_history(:, 2);
    velocity = X_history(:, 7);
    
    % Calcular métricas
    n_ss = round(0.1 * length(t_history));
    x_ss = mean(position_x(end-n_ss:end));
    error_ss = abs(X_step - x_ss);
    
    % Rise time
    x_10 = 0.1 * X_step;
    x_90 = 0.9 * X_step;
    idx_10 = find(position_x >= x_10, 1, 'first');
    idx_90 = find(position_x >= x_90, 1, 'first');
    if ~isempty(idx_10) && ~isempty(idx_90)
        rise_time = t_history(idx_90) - t_history(idx_10);
    else
        rise_time = NaN;
    end
    
    % Settling time
    tolerance = 0.02;
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
    
    % Overshoot
    x_max = max(position_x);
    overshoot_pct = max(0, ((x_max - X_step) / X_step) * 100);
    
    % IAE
    IAE = trapz(t_history, abs(error_history));
    
    % Guardar resultados
    all_results{cfg}.name = config_name;
    all_results{cfg}.t = t_history;
    all_results{cfg}.position_x = position_x;
    all_results{cfg}.position_y = position_y;
    all_results{cfg}.velocity = velocity;
    all_results{cfg}.error = error_history;
    all_results{cfg}.torques = U_history;
    all_results{cfg}.x_ss = x_ss;
    all_results{cfg}.error_ss = error_ss;
    all_results{cfg}.rise_time = rise_time;
    all_results{cfg}.settling_time = settling_time;
    all_results{cfg}.overshoot_pct = overshoot_pct;
    all_results{cfg}.IAE = IAE;
    
    fprintf('    ✓ Completado: ess=%.4fm, tr=%.2fs, ts=%.2fs, Mp=%.1f%%\n\n', ...
        error_ss, rise_time, settling_time, overshoot_pct);
end

%% ═══════════════════════════════════════════════════════════════════════
%%  7. TABLA COMPARATIVA
%% ═══════════════════════════════════════════════════════════════════════

fprintf('══════════════════════════════════════════════════════════════\n');
fprintf('              TABLA COMPARATIVA DE RESULTADOS                 \n');
fprintf('══════════════════════════════════════════════════════════════\n\n');

fprintf('┌─────────────────────┬───────────┬───────────┬───────────┐\n');
fprintf('│       Métrica       │Solo Fuzzy │Fuzzy+PID  │ Completo  │\n');
fprintf('├─────────────────────┼───────────┼───────────┼───────────┤\n');
fprintf('│ Pos. Final (m)      │  %.4f   │  %.4f   │  %.4f   │\n', ...
    all_results{1}.x_ss, all_results{2}.x_ss, all_results{3}.x_ss);
fprintf('│ Error ess (m)       │  %.4f   │  %.4f   │  %.4f   │\n', ...
    all_results{1}.error_ss, all_results{2}.error_ss, all_results{3}.error_ss);
fprintf('│ Rise Time (s)       │  %.4f   │  %.4f   │  %.4f   │\n', ...
    all_results{1}.rise_time, all_results{2}.rise_time, all_results{3}.rise_time);
fprintf('│ Settling Time (s)   │  %.4f   │  %.4f   │  %.4f   │\n', ...
    all_results{1}.settling_time, all_results{2}.settling_time, all_results{3}.settling_time);
fprintf('│ Overshoot (%%)       │  %.2f    │  %.2f    │  %.2f    │\n', ...
    all_results{1}.overshoot_pct, all_results{2}.overshoot_pct, all_results{3}.overshoot_pct);
fprintf('│ IAE (m·s)           │  %.4f   │  %.4f   │  %.4f   │\n', ...
    all_results{1}.IAE, all_results{2}.IAE, all_results{3}.IAE);
fprintf('└─────────────────────┴───────────┴───────────┴───────────┘\n\n');

%% ═══════════════════════════════════════════════════════════════════════
%%  8. VISUALIZACIÓN COMPARATIVA
%% ═══════════════════════════════════════════════════════════════════════

figure('Name', 'Comparación Step Response - 3 Configuraciones', 'Position', [50 50 1500 900]);

% Subplot 1: Posición X (Respuesta Principal)
subplot(2,3,1);
hold on;
for cfg = 1:3
    plot(all_results{cfg}.t, all_results{cfg}.position_x, ...
        line_styles{cfg}, 'Color', colors{cfg}, 'LineWidth', 2, ...
        'DisplayName', all_results{cfg}.name);
end
yline(X_step, 'k--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
fill([0 max_time max_time 0], [band_lower band_lower band_upper band_upper], ...
    'g', 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
xlabel('Tiempo (s)');
ylabel('Posición X (m)');
title('Respuesta de Posición al Escalón');
legend('Location', 'best');
grid on;
xlim([0 max_time]);

% Subplot 2: Error de seguimiento
subplot(2,3,2);
hold on;
for cfg = 1:3
    plot(all_results{cfg}.t, all_results{cfg}.error, ...
        line_styles{cfg}, 'Color', colors{cfg}, 'LineWidth', 1.5, ...
        'DisplayName', all_results{cfg}.name);
end
yline(0, 'k--', 'HandleVisibility', 'off');
xlabel('Tiempo (s)');
ylabel('Error de Posición (m)');
title('Error de Seguimiento');
legend('Location', 'best');
grid on;
xlim([0 max_time]);

% Subplot 3: Zoom Estado Estacionario (últimos 10s)
subplot(2,3,3);
hold on;
t_zoom = max_time - 10;
for cfg = 1:3
    idx = all_results{cfg}.t >= t_zoom;
    plot(all_results{cfg}.t(idx), all_results{cfg}.position_x(idx), ...
        line_styles{cfg}, 'Color', colors{cfg}, 'LineWidth', 2, ...
        'DisplayName', sprintf('%s (ess=%.3fm)', all_results{cfg}.name, all_results{cfg}.error_ss));
end
yline(X_step, 'k--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
fill([t_zoom max_time max_time t_zoom], [band_lower band_lower band_upper band_upper], ...
    'g', 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off');
xlabel('Tiempo (s)');
ylabel('Posición X (m)');
title('ZOOM: Estado Estacionario');
legend('Location', 'best');
grid on;

% Subplot 4: Trayectoria XY
subplot(2,3,4);
hold on;
for cfg = 1:3
    plot(all_results{cfg}.position_x, all_results{cfg}.position_y, ...
        line_styles{cfg}, 'Color', colors{cfg}, 'LineWidth', 1.5, ...
        'DisplayName', all_results{cfg}.name);
end
plot(0, 0, 'ko', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Inicio');
plot(X_step, Y_step, 'r*', 'MarkerSize', 15, 'LineWidth', 2, 'DisplayName', 'Objetivo');
xlabel('X (m)'); ylabel('Y (m)');
title('Trayectoria XY');
legend('Location', 'best');
grid on;
axis equal;

% Subplot 5: Velocidad
subplot(2,3,5);
hold on;
for cfg = 1:3
    plot(all_results{cfg}.t, all_results{cfg}.velocity, ...
        line_styles{cfg}, 'Color', colors{cfg}, 'LineWidth', 1.5, ...
        'DisplayName', all_results{cfg}.name);
end
xlabel('Tiempo (s)');
ylabel('Velocidad (m/s)');
title('Velocidad Longitudinal');
legend('Location', 'best');
grid on;
xlim([0 max_time]);

% Subplot 6: Barras Comparativas
subplot(2,3,6);
metrics = {'ess (m)', 'tr (s)', 'IAE'};
values = [
    all_results{1}.error_ss, all_results{2}.error_ss, all_results{3}.error_ss;
    all_results{1}.rise_time, all_results{2}.rise_time, all_results{3}.rise_time;
    all_results{1}.IAE, all_results{2}.IAE, all_results{3}.IAE;
    ];
x_pos = 1:3;
bar_width = 0.25;
hold on;
for cfg = 1:3
    bar(x_pos + (cfg-2)*bar_width, values(:, cfg), bar_width, ...
        'FaceColor', colors{cfg}, 'DisplayName', configs{cfg, 1});
end
xticks(1:3);
xticklabels(metrics);
ylabel('Valor');
title('Comparación de Métricas');
legend('Location', 'best');
grid on;

% Título general
sgtitle('COMPARACIÓN: Solo Fuzzy vs Fuzzy+PID vs Sistema Completo', ...
    'FontSize', 14, 'FontWeight', 'bold');

%% 9. GUARDAR RESULTADOS
results.configs = configs;
results.all_results = all_results;
results.X_step = X_step;
save('closedloop_comparison_results.mat', 'results');
fprintf('✓ Resultados guardados en closedloop_comparison_results.mat\n\n');

fprintf('══════════════════════════════════════════════════════════════\n');
fprintf('         TEST COMPARATIVO COMPLETADO                          \n');
fprintf('══════════════════════════════════════════════════════════════\n\n');
