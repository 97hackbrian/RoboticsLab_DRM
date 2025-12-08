function setup_paths()
% setup_paths: Configura las rutas del proyecto automáticamente
%
% USO:
%   Ejecutar una vez al inicio de cada sesión MATLAB:
%   >> setup_paths
%
% O agregar a startup.m para carga automática

% Obtener directorio raíz del proyecto
current_dir = fileparts(mfilename('fullpath'));

% Agregar subdirectorios al path
addpath(fullfile(current_dir, 'config'));
addpath(fullfile(current_dir, 'system'));
addpath(fullfile(current_dir, 'controllers'));
addpath(fullfile(current_dir, 'tests'));
addpath(fullfile(current_dir, 'utils'));

fprintf('✓ Rutas del proyecto configuradas correctamente.\n');
fprintf('  - config/\n');
fprintf('  - system/\n');
fprintf('  - controllers/\n');
fprintf('  - tests/\n');
fprintf('  - utils/\n\n');
fprintf('Puedes ejecutar ahora:\n');
fprintf('  >> run_closed_loop_simulation\n');
fprintf('  >> cd system; open_loop_simulation\n');
fprintf('  >> cd tests; test_gravity_compensation\n\n');
end
