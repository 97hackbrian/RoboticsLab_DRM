% run_system.m
% Punto de entrada principal para la simulación del robot Skid-Steer
% Ejecuta la simulación en lazo abierto (sin controlador)
%
% USO:
%   Desde terminal: ~/MATLAB/R2023a/bin/./matlab -batch run_system
%   Desde MATLAB:   >> run_system

clear; clc; close all;

fprintf('\n');
fprintf('╔════════════════════════════════════════════════════════════╗\n');
fprintf('║   SIMULACIÓN ROBOT SKID-STEER 4WD - LAZO ABIERTO          ║\n');
fprintf('╚════════════════════════════════════════════════════════════╝\n');
fprintf('\n');

% Obtener directorio raíz del proyecto
project_root = fileparts(mfilename('fullpath'));

% Configurar rutas del proyecto
fprintf('► Configurando rutas del proyecto...\n');
addpath(fullfile(project_root, 'config'));
addpath(fullfile(project_root, 'system'));
addpath(fullfile(project_root, 'controllers'));
addpath(fullfile(project_root, 'tests'));
addpath(fullfile(project_root, 'utils'));
fprintf('  ✓ Rutas configuradas correctamente\n\n');

% Ejecutar simulación en lazo abierto
fprintf('► Ejecutando simulación en lazo abierto...\n');
fprintf('  Modelo: Robot Skid-Steer (4 ruedas)\n');
fprintf('  Estado: 12 variables [x,y,z,φ,θ,ψ,u,v,w,p,q,r]\n');
fprintf('  Física: Dinámica 3D + Gravedad + Fricción neumático-terreno\n');
fprintf('  Sensores: IMU + Encoders + Cámara (con ruido)\n\n');

try
    % Cambiar al directorio system y ejecutar
    cd(fullfile(project_root, 'system'));
    open_loop_simulation;
    
    fprintf('\n► Simulación completada exitosamente ✓\n');
    fprintf('  Las gráficas muestran el comportamiento del sistema físico\n');
    fprintf('  sin controlador (solo torque constante en pendiente).\n\n');
    
catch ME
    fprintf('\n✗ ERROR durante la simulación:\n');
    fprintf('  Mensaje: %s\n', ME.message);
    fprintf('  Archivo: %s\n', ME.stack(1).file);
    fprintf('  Línea: %d\n\n', ME.stack(1).line);
    rethrow(ME);
end

% Volver al directorio raíz
cd(project_root);

fprintf('╔════════════════════════════════════════════════════════════╗\n');
fprintf('║   SIMULACIÓN FINALIZADA                                    ║\n');
fprintf('╚════════════════════════════════════════════════════════════╝\n');
fprintf('\n');
