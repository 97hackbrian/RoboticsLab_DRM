% run_fuzzy_control.m
% Punto de entrada para ejecutar simulación con controlador difuso
%
% USO:
%   Desde terminal: ~/MATLAB/R2023a/bin/./matlab -batch run_fuzzy_control
%   Desde MATLAB:   >> run_fuzzy_control

clear; clc; close all;

fprintf('\n');
fprintf('╔════════════════════════════════════════════════════════════╗\n');
fprintf('║   CONTROL DIFUSO - TRACKING DE POSICIÓN                   ║\n');
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

% Ejecutar simulación en lazo cerrado
fprintf('► Lanzando simulación en lazo cerrado...\n\n');

% Guardar directorio actual
current_dir = pwd;

try
    % Cambiar al directorio system y ejecutar
    cd(fullfile(project_root, 'system'));
    waypoint_navigation_simulation;
    
    % Volver al directorio raíz
    cd(current_dir);
    
catch ME
    % Volver al directorio raíz en caso de error
    cd(current_dir);
    
    fprintf('\n✗ ERROR durante la simulación:\n');
    fprintf('  Mensaje: %s\n', ME.message);
    if ~isempty(ME.stack)
        fprintf('  Archivo: %s\n', ME.stack(1).file);
        fprintf('  Línea: %d\n\n', ME.stack(1).line);
    end
    rethrow(ME);
end

fprintf('╔════════════════════════════════════════════════════════════╗\n');
fprintf('║   SIMULACIÓN FINALIZADA                                    ║\n');
fprintf('╚════════════════════════════════════════════════════════════╝\n');
fprintf('\n');
