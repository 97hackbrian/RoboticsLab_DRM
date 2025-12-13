% test_rrt_star_algorithm.m
% Tests unitarios del algoritmo RRT*
% Verifica correcta generación de paths en diferentes escenarios

function test_rrt_star_algorithm()
clc;

%% Agregar paths
addpath('../config');
addpath('../planners');
addpath('../utils');

fprintf('\n╔════════════════════════════════════════════════════════════════╗\n');
fprintf('║              TESTS DEL ALGORITMO RRT*                         ║\n');
fprintf('╚════════════════════════════════════════════════════════════════╝\n\n');

%% Configurar tests
tests_passed = 0;
tests_failed = 0;
test_results = {};

%% ═══════════════════════════════════════════════════════════════════════
%  TEST 1: Entorno vacío
%% ═══════════════════════════════════════════════════════════════════════
fprintf('═══════════════════════════════════════════════════════════════\n');
fprintf('TEST 1: Entorno vacío\n');
fprintf('═══════════════════════════════════════════════════════════════\n');

map = environment_map('empty');
params = planner_parameters();
params.show_progress = false;
params.max_iterations = 1000;

[path, ~, stats] = rrt_star(map.start, map.goal, map, params);

if ~isempty(path) && size(path, 1) >= 2
    tests_passed = tests_passed + 1;
    test_results{end+1} = 'PASS: Entorno vacío - Path encontrado';
    fprintf('✓ PASSED: Path encontrado con %d puntos\n', size(path, 1));
else
    tests_failed = tests_failed + 1;
    test_results{end+1} = 'FAIL: Entorno vacío - No se encontró path';
    fprintf('✗ FAILED: No se encontró path\n');
end

%% ═══════════════════════════════════════════════════════════════════════
%  TEST 2: Obstáculo único
%% ═══════════════════════════════════════════════════════════════════════
fprintf('\n═══════════════════════════════════════════════════════════════\n');
fprintf('TEST 2: Obstáculo único\n');
fprintf('═══════════════════════════════════════════════════════════════\n');

map.obstacles = {struct('type', 'circle', 'center', [10; 10], 'radius', 3)};
map.start = [2; 2];
map.goal = [18; 18];

[path, ~, ~] = rrt_star(map.start, map.goal, map, params);

% Verificar que el path evita el obstáculo
path_valid = true;
if ~isempty(path)
    for i = 1:size(path, 1)
        dist = norm(path(i,:)' - [10; 10]);
        if dist < 3 + map.robot_radius
            path_valid = false;
            break;
        end
    end
end

if ~isempty(path) && path_valid
    tests_passed = tests_passed + 1;
    test_results{end+1} = 'PASS: Obstáculo único - Path válido';
    fprintf('✓ PASSED: Path evita obstáculo correctamente\n');
else
    tests_failed = tests_failed + 1;
    test_results{end+1} = 'FAIL: Obstáculo único - Path inválido o no encontrado';
    fprintf('✗ FAILED: Path atraviesa obstáculo o no se encontró\n');
end

%% ═══════════════════════════════════════════════════════════════════════
%  TEST 3: Múltiples obstáculos (escenario simple)
%% ═══════════════════════════════════════════════════════════════════════
fprintf('\n═══════════════════════════════════════════════════════════════\n');
fprintf('TEST 3: Múltiples obstáculos\n');
fprintf('═══════════════════════════════════════════════════════════════\n');

map = environment_map('simple');
params.max_iterations = 2000;

[path, ~, stats] = rrt_star(map.start, map.goal, map, params);

if ~isempty(path) && stats.goal_found_iter > 0
    tests_passed = tests_passed + 1;
    test_results{end+1} = 'PASS: Múltiples obstáculos - Path encontrado';
    fprintf('✓ PASSED: Path encontrado en iteración %d\n', stats.goal_found_iter);
else
    tests_failed = tests_failed + 1;
    test_results{end+1} = 'FAIL: Múltiples obstáculos - No se encontró path';
    fprintf('✗ FAILED: No se encontró path\n');
end

%% ═══════════════════════════════════════════════════════════════════════
%  TEST 4: Path es libre de colisiones
%% ═══════════════════════════════════════════════════════════════════════
fprintf('\n═══════════════════════════════════════════════════════════════\n');
fprintf('TEST 4: Verificación de colisiones en path\n');
fprintf('═══════════════════════════════════════════════════════════════\n');

if ~isempty(path)
    collision_free = true;
    for i = 1:(size(path, 1) - 1)
        p1 = path(i, :)';
        p2 = path(i+1, :)';
        if collision_check(p1, p2, map.obstacles, map.robot_radius)
            collision_free = false;
            break;
        end
    end
    
    if collision_free
        tests_passed = tests_passed + 1;
        test_results{end+1} = 'PASS: Path libre de colisiones';
        fprintf('✓ PASSED: Path completamente libre de colisiones\n');
    else
        tests_failed = tests_failed + 1;
        test_results{end+1} = 'FAIL: Path tiene colisiones';
        fprintf('✗ FAILED: Path tiene segmentos en colisión\n');
    end
else
    tests_failed = tests_failed + 1;
    test_results{end+1} = 'FAIL: No hay path para verificar colisiones';
    fprintf('✗ SKIPPED: No hay path para verificar\n');
end

%% ═══════════════════════════════════════════════════════════════════════
%  TEST 5: Conectividad inicio-meta
%% ═══════════════════════════════════════════════════════════════════════
fprintf('\n═══════════════════════════════════════════════════════════════\n');
fprintf('TEST 5: Conectividad inicio-meta\n');
fprintf('═══════════════════════════════════════════════════════════════\n');

if ~isempty(path)
    start_match = norm(path(1,:)' - map.start) < 0.1;
    goal_match = norm(path(end,:)' - map.goal) < params.goal_tolerance + 0.5;
    
    if start_match && goal_match
        tests_passed = tests_passed + 1;
        test_results{end+1} = 'PASS: Path conecta inicio y meta';
        fprintf('✓ PASSED: Path conecta correctamente inicio y meta\n');
    else
        tests_failed = tests_failed + 1;
        test_results{end+1} = 'FAIL: Path no conecta inicio/meta';
        fprintf('✗ FAILED: Path no conecta inicio (%.2f) o meta (%.2f)\n', ...
            norm(path(1,:)' - map.start), norm(path(end,:)' - map.goal));
    end
else
    tests_failed = tests_failed + 1;
    fprintf('✗ SKIPPED: No hay path para verificar\n');
end

%% ═══════════════════════════════════════════════════════════════════════
%  TEST 6: Suavizado de path
%% ═══════════════════════════════════════════════════════════════════════
fprintf('\n═══════════════════════════════════════════════════════════════\n');
fprintf('TEST 6: Suavizado de path\n');
fprintf('═══════════════════════════════════════════════════════════════\n');

if ~isempty(path)
    smooth_path = path_smoothing(path, map.obstacles, map.robot_radius, params);
    
    % El path suavizado debe tener menos o igual puntos
    if size(smooth_path, 1) <= size(path, 1)
        tests_passed = tests_passed + 1;
        test_results{end+1} = 'PASS: Suavizado reduce/mantiene puntos';
        fprintf('✓ PASSED: Path suavizado tiene %d puntos (original: %d)\n', ...
            size(smooth_path, 1), size(path, 1));
    else
        tests_failed = tests_failed + 1;
        test_results{end+1} = 'FAIL: Suavizado aumentó puntos (anómalo)';
        fprintf('✗ WARNING: Path suavizado tiene más puntos que original\n');
    end
else
    tests_failed = tests_failed + 1;
    fprintf('✗ SKIPPED: No hay path para suavizar\n');
end

%% ═══════════════════════════════════════════════════════════════════════
%  RESUMEN DE TESTS
%% ═══════════════════════════════════════════════════════════════════════
fprintf('\n\n╔════════════════════════════════════════════════════════════════╗\n');
fprintf('║                   RESUMEN DE TESTS                            ║\n');
fprintf('╠════════════════════════════════════════════════════════════════╣\n');
fprintf('║  Tests pasados:  %d                                           ║\n', tests_passed);
fprintf('║  Tests fallidos: %d                                           ║\n', tests_failed);
fprintf('║  Total:          %d                                           ║\n', tests_passed + tests_failed);
fprintf('╚════════════════════════════════════════════════════════════════╝\n\n');

for i = 1:length(test_results)
    fprintf('  %s\n', test_results{i});
end

fprintf('\n');

if tests_failed == 0
    fprintf('✓✓✓ TODOS LOS TESTS PASARON ✓✓✓\n\n');
else
    fprintf('✗✗✗ ALGUNOS TESTS FALLARON ✗✗✗\n\n');
end

end
