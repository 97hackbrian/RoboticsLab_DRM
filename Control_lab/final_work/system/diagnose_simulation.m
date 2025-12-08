% diagnose_simulation.m
% Script para diagnosticar problemas en la simulación
% Ejecuta una simulación corta y analiza los estados intermedios

clear; clc; close all;

% Agregar rutas
addpath('../config');
addpath('../system');

%% Cargar parámetros
params = robot_parameters();

%% Condiciones iniciales más conservadoras
% Pequeña pendiente inicial
theta_0 = deg2rad(2); % Reducido de 5° a 2°
X0 = [0; 0; 0;  0; theta_0; 0;  0; 0; 0;  0; 0; 0];

%% Entrada de control reducida
% Torque más pequeño para diagnóstico
u_control = [1.5; 1.5; 1.5; 1.5]; % Reducido de 3 Nm a 1.5 Nm

%% Simulación MUY CORTA con debug
t_span = [0 1]; % Solo 1 segundo

model_func = @(t,x) skid_steer_robot_model(t, x, u_control, params);

% Opciones más estrictas para detectar problemas
opts = odeset('RelTol', 1e-7, 'AbsTol', 1e-9, 'MaxStep', 0.005);

fprintf('\n============ DIAGNÓSTICO DE SIMULACIÓN ============\n');
fprintf('Condiciones iniciales:\n');
fprintf('  Pitch inicial: %.2f°\n', rad2deg(theta_0));
fprintf('  Torque: %.2f Nm\n', u_control(1));
fprintf('  Tiempo: %.1f s\n\n', t_span(2));

try
    [t, X_real] = ode45(model_func, t_span, X0, opts);
    
    fprintf('✓ Simulación completada\n\n');
    
    % Análisis de resultados
    fprintf('Análisis de estados:\n');
    fprintf('  Posición X: %.3f m (inicial) → %.3f m (final)\n', X0(1), X_real(end,1));
    fprintf('  Pitch: %.2f° (inicial) → %.2f° (final)\n', rad2deg(X0(5)), rad2deg(X_real(end,5)));
    fprintf('  Velocidad u: %.3f m/s (inicial) → %.3f m/s (final)\n', X0(7), X_real(end,7));
    fprintf('  Roll rate p: %.4f rad/s (inicial) → %.4f rad/s (final)\n', X0(10), X_real(end,10));
    fprintf('  Pitch rate q: %.4f rad/s (inicial) → %.4f rad/s (final)\n', X0(11), X_real(end,11));
    fprintf('  Yaw rate r: %.4f rad/s (inicial) → %.4f rad/s (final)\n\n', X0(12), X_real(end,12));
    
    % Verificar si hay valores anormales
    fprintf('Verificación de estabilidad:\n');
    if any(isnan(X_real(:))) || any(isinf(X_real(:)))
        fprintf('  ✗ CRÍTICO: Se detectaron valores NaN o Inf\n');
    else
        fprintf('  ✓ Sin valores NaN o Inf\n');
    end
    
    max_pitch_rate = max(abs(X_real(:,11)));
    if max_pitch_rate > 1.0 % rad/s
        fprintf('  ⚠ Pitch rate excesivo: %.2f rad/s (> 1.0 rad/s)\n', max_pitch_rate);
    else
        fprintf('  ✓ Pitch rate razonable: %.4f rad/s\n', max_pitch_rate);
    end
    
    max_pitch = max(abs(rad2deg(X_real(:,5))));
    if max_pitch > 15
        fprintf('  ⚠ Pitch angle excesivo: %.2f° (> 15°)\n', max_pitch);
    else
        fprintf('  ✓ Pitch angle razonable: %.2f°\n', max_pitch);
    end
    
    % Gráfico de diagnóstico
    figure('Name', 'Diagnóstico de Estado', 'Position', [100 100 1200 600]);
    
    subplot(2,3,1);
    plot(t, X_real(:,1), 'LineWidth', 2);
    title('Posición X'); xlabel('t (s)'); ylabel('X (m)'); grid on;
    
    subplot(2,3,2);
    plot(t, rad2deg(X_real(:,5)), 'LineWidth', 2);
    title('Pitch Angle'); xlabel('t (s)'); ylabel('Pitch (°)'); grid on;
    
    subplot(2,3,3);
    plot(t, X_real(:,7), 'LineWidth', 2);
    title('Velocidad Longitudinal'); xlabel('t (s)'); ylabel('u (m/s)'); grid on;
    
    subplot(2,3,4);
    plot(t, X_real(:,10:12), 'LineWidth', 2);
    title('Velocidades Angulares'); xlabel('t (s)'); ylabel('rad/s');
    legend('p (roll)', 'q (pitch)', 'r (yaw)'); grid on;
    
    subplot(2,3,5);
    plot(t, rad2deg(X_real(:,4:6)), 'LineWidth', 2);
    title('Ángulos de Euler'); xlabel('t (s)'); ylabel('Grados');
    legend('\phi (roll)', '\theta (pitch)', '\psi (yaw)'); grid on;
    
    subplot(2,3,6);
    plot3(X_real(:,1), X_real(:,2), X_real(:,3), 'LineWidth', 2);
    title('Trayectoria 3D'); xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    grid on; axis equal;
    
catch ME
    fprintf('✗ ERROR: %s\n', ME.message);
    fprintf('  Archivo: %s\n', ME.stack(1).file);
    fprintf('  Línea: %d\n', ME.stack(1).line);
end

fprintf('===================================================\n\n');
