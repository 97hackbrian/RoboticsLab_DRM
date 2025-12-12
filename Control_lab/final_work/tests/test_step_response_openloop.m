% test_step_response_openloop.m
% Prueba de respuesta al escalón en LAZO ABIERTO
% Evalúa el sistema sin controlador aplicando un step unitario de torque
%
% MÉTRICAS DE DESEMPEÑO:
%   - Tiempo de subida (tr): 10% a 90% del valor final
%   - Tiempo de asentamiento (ts): ±2% del valor final
%   - Sobrepaso máximo (Mp): % sobre valor final
%   - Valor en estado estacionario

clear; clc; close all;

% Agregar rutas
addpath('../config');
addpath('../system');

fprintf('\n╔════════════════════════════════════════════════════════════╗\n');
fprintf('║   TEST DE RESPUESTA AL ESCALÓN - LAZO ABIERTO             ║\n');
fprintf('╚════════════════════════════════════════════════════════════╝\n\n');

%% 1. CARGAR PARÁMETROS
params = robot_parameters();

% ═══════════════════════════════════════════════════════════════════════
%  AJUSTE DE FRICCIÓN PARA SIMULACIÓN REALISTA
% ═══════════════════════════════════════════════════════════════════════
%  El Cd original (0.1) es muy bajo para un robot terrestre.
%  Con τ = 1 Nm y Cd = 0.1, la velocidad terminal sería ~40 m/s (irreal).
%
%  Velocidad terminal: v_ss = sqrt(F_total / Cd) donde F = τ/r
%  Para un robot que alcance ~1-2 m/s con τ = 1 Nm:
%    Cd = F / v² = (4 * 1/0.2) / (1.5)² ≈ 9
%
%  Aumentamos Cd para una simulación coherente:
params.Cd = 5.0;  % Fricción viscosa ajustada (original: 0.1)

%% 2. CONDICIONES INICIALES
% Robot en reposo, plano (sin pendiente)
X0 = zeros(12, 1);

%% 3. ENTRADA DE CONTROL: ESCALÓN UNITARIO
% Aplicamos torque constante de 1 Nm a las 4 ruedas simultáneamente
% Esto equivale a un "step" en la entrada del sistema
tau_step = 1.0; % Nm (escalón unitario)
u_control = [tau_step; tau_step; tau_step; tau_step];

%% 4. SIMULACIÓN
t_sim = 10; % segundos (suficiente para alcanzar estado estacionario)
t_span = [0 t_sim];

model_func = @(t,x) skid_steer_robot_model(t, x, u_control, params);

% Opciones de integración
opts = odeset('RelTol', 1e-7, 'AbsTol', 1e-9, 'MaxStep', 0.01);

fprintf('► Aplicando escalón de τ = %.2f Nm a todas las ruedas...\n', tau_step);
fprintf('► Simulando durante %.1f segundos...\n\n', t_sim);

try
    [t, X] = ode45(model_func, t_span, X0, opts);
    fprintf('✓ Simulación completada (%d puntos)\n\n', length(t));
catch ME
    fprintf('✗ ERROR: %s\n', ME.message);
    return;
end

%% 5. EXTRAER RESPUESTAS
% Respuesta principal: velocidad longitudinal (u)
velocity = X(:, 7); % m/s

% Otras respuestas de interés
position_x = X(:, 1); % m
pitch = rad2deg(X(:, 5)); % grados
yaw_rate = X(:, 12); % rad/s

%% 6. CALCULAR MÉTRICAS DE DESEMPEÑO

% Valor en estado estacionario (promedio de últimas muestras)
n_ss = round(0.1 * length(t)); % Último 10% de la simulación
v_ss = mean(velocity(end-n_ss:end));

fprintf('══════════════════════════════════════════════════════════════\n');
fprintf('                 MÉTRICAS DE DESEMPEÑO                        \n');
fprintf('══════════════════════════════════════════════════════════════\n\n');

fprintf('► Valor en Estado Estacionario:\n');
fprintf('    Velocidad (v_ss): %.4f m/s\n', v_ss);
fprintf('    Posición final X: %.3f m\n\n', position_x(end));

% --- TIEMPO DE SUBIDA (Rise Time) ---
% Tiempo para ir de 10% a 90% del valor final
v_10 = 0.1 * v_ss;
v_90 = 0.9 * v_ss;

idx_10 = find(velocity >= v_10, 1, 'first');
idx_90 = find(velocity >= v_90, 1, 'first');

if ~isempty(idx_10) && ~isempty(idx_90)
    t_10 = t(idx_10);
    t_90 = t(idx_90);
    rise_time = t_90 - t_10;
else
    t_10 = NaN; t_90 = NaN;
    rise_time = NaN;
end

fprintf('► Tiempo de Subida (Rise Time, tr):\n');
fprintf('    t(10%%): %.4f s\n', t_10);
fprintf('    t(90%%): %.4f s\n', t_90);
fprintf('    tr = t(90%%) - t(10%%) = %.4f s\n\n', rise_time);

% --- TIEMPO DE ASENTAMIENTO (Settling Time) ---
% Tiempo para que la respuesta permanezca dentro de ±2% del valor final
tolerance = 0.02; % 2%
band_upper = v_ss * (1 + tolerance);
band_lower = v_ss * (1 - tolerance);

% Buscar desde el final hacia atrás
settling_time = NaN;
for i = length(t):-1:1
    if velocity(i) > band_upper || velocity(i) < band_lower
        if i < length(t)
            settling_time = t(i+1);
        end
        break;
    end
end

% Si toda la respuesta está dentro de la banda
if i == 1 && velocity(1) >= band_lower && velocity(1) <= band_upper
    settling_time = 0;
end

fprintf('► Tiempo de Asentamiento (Settling Time, ts):\n');
fprintf('    Banda: ±%.0f%% del valor final\n', tolerance*100);
fprintf('    ts = %.4f s\n\n', settling_time);

% --- SOBREPASO MÁXIMO (Overshoot) ---
v_max = max(velocity);
if v_ss > 0
    overshoot_pct = ((v_max - v_ss) / v_ss) * 100;
else
    overshoot_pct = 0;
end

% Tiempo del pico
[~, idx_peak] = max(velocity);
t_peak = t(idx_peak);

fprintf('► Sobrepaso Máximo (Overshoot, Mp):\n');
fprintf('    Valor máximo: %.4f m/s\n', v_max);
fprintf('    Tiempo de pico: %.4f s\n', t_peak);
fprintf('    Mp = %.2f %%\n\n', overshoot_pct);

% --- RESUMEN ---
fprintf('══════════════════════════════════════════════════════════════\n');
fprintf('                    RESUMEN DE MÉTRICAS                        \n');
fprintf('══════════════════════════════════════════════════════════════\n');
fprintf('┌─────────────────────────────┬───────────────────────────────┐\n');
fprintf('│ Parámetro                   │ Valor                         │\n');
fprintf('├─────────────────────────────┼───────────────────────────────┤\n');
fprintf('│ Entrada (τ)                 │ %.2f Nm                       │\n', tau_step);
fprintf('│ Velocidad Estado Est. (v_ss)│ %.4f m/s                     │\n', v_ss);
fprintf('│ Tiempo de Subida (tr)       │ %.4f s                       │\n', rise_time);
fprintf('│ Tiempo de Asentamiento (ts) │ %.4f s                       │\n', settling_time);
fprintf('│ Sobrepaso Máximo (Mp)       │ %.2f %%                       │\n', overshoot_pct);
fprintf('│ Tiempo de Pico (tp)         │ %.4f s                       │\n', t_peak);
fprintf('└─────────────────────────────┴───────────────────────────────┘\n\n');

%% 7. VISUALIZACIÓN
figure('Name', 'Respuesta al Escalón - Lazo Abierto', 'Position', [50 50 1400 800]);

% Subplot 1: Velocidad (Respuesta Principal)
subplot(2,3,1);
plot(t, velocity, 'b-', 'LineWidth', 2);
hold on;
% Líneas de referencia
yline(v_ss, 'r--', 'LineWidth', 1.5, 'Label', sprintf('v_{ss} = %.3f m/s', v_ss));
yline(v_90, 'g:', 'LineWidth', 1, 'Label', '90%');
yline(v_10, 'm:', 'LineWidth', 1, 'Label', '10%');
% Marcar tiempo de subida
if ~isnan(rise_time)
    plot([t_10 t_10], [0 v_10], 'g--');
    plot([t_90 t_90], [0 v_90], 'g--');
end
xlabel('Tiempo (s)');
ylabel('Velocidad (m/s)');
title('Respuesta de Velocidad al Escalón');
legend('v(t)', 'Estado Estacionario', 'Location', 'best');
grid on;

% Subplot 2: Posición
subplot(2,3,2);
plot(t, position_x, 'k-', 'LineWidth', 2);
xlabel('Tiempo (s)');
ylabel('Posición X (m)');
title('Posición Longitudinal');
grid on;

% Subplot 3: Pitch y Roll
subplot(2,3,3);
plot(t, rad2deg(X(:,4)), 'r-', 'LineWidth', 1.5); hold on;
plot(t, rad2deg(X(:,5)), 'b-', 'LineWidth', 1.5);
plot(t, rad2deg(X(:,6)), 'g-', 'LineWidth', 1.5);
xlabel('Tiempo (s)');
ylabel('Ángulo (°)');
title('Ángulos de Euler');
legend('φ (Roll)', 'θ (Pitch)', 'ψ (Yaw)', 'Location', 'best');
grid on;

% Subplot 4: Velocidades angulares
subplot(2,3,4);
plot(t, X(:,10), 'r-', 'LineWidth', 1.5); hold on;
plot(t, X(:,11), 'b-', 'LineWidth', 1.5);
plot(t, X(:,12), 'g-', 'LineWidth', 1.5);
xlabel('Tiempo (s)');
ylabel('Velocidad Angular (rad/s)');
title('Velocidades Angulares');
legend('p (Roll)', 'q (Pitch)', 'r (Yaw)', 'Location', 'best');
grid on;

% Subplot 5: Respuesta con banda de asentamiento
subplot(2,3,5);
plot(t, velocity, 'b-', 'LineWidth', 2);
hold on;
fill([0 t_sim t_sim 0], [band_lower band_lower band_upper band_upper], ...
    'g', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
yline(v_ss, 'r--', 'LineWidth', 1.5);
if ~isnan(settling_time)
    xline(settling_time, 'k--', 'LineWidth', 1.5, 'Label', sprintf('ts = %.2fs', settling_time));
end
xlabel('Tiempo (s)');
ylabel('Velocidad (m/s)');
title(sprintf('Tiempo de Asentamiento (±2%%) = %.3f s', settling_time));
grid on;

% Subplot 6: Trayectoria 3D
subplot(2,3,6);
plot3(X(:,1), X(:,2), X(:,3), 'b-', 'LineWidth', 2);
hold on;
plot3(X(1,1), X(1,2), X(1,3), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot3(X(end,1), X(end,2), X(end,3), 'rs', 'MarkerSize', 10, 'LineWidth', 2);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Trayectoria 3D');
legend('Trayectoria', 'Inicio', 'Fin', 'Location', 'best');
grid on;
axis equal;

% Título general
sgtitle(sprintf('RESPUESTA AL ESCALÓN - LAZO ABIERTO (τ = %.1f Nm)', tau_step), ...
    'FontSize', 14, 'FontWeight', 'bold');

%% 8. GUARDAR RESULTADOS
results.tau_step = tau_step;
results.v_ss = v_ss;
results.rise_time = rise_time;
results.settling_time = settling_time;
results.overshoot_pct = overshoot_pct;
results.peak_time = t_peak;
results.t = t;
results.velocity = velocity;
results.position = position_x;

% Guardar en archivo
save('openloop_step_results.mat', 'results');
fprintf('✓ Resultados guardados en openloop_step_results.mat\n\n');

fprintf('══════════════════════════════════════════════════════════════\n');
fprintf('         TEST DE LAZO ABIERTO COMPLETADO                      \n');
fprintf('══════════════════════════════════════════════════════════════\n\n');
