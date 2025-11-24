%% Generación del Dataset para el Helicóptero 3D - RELACIÓN CORREGIDAv1
%clear; clc; close all;

% Parámetros del sistema (debes definir estos valores según tu sistema)
% Estos son valores típicos - ajústalos según tu configuración real




%% ############### MODELING ###############
% These parameters are used for model representation and controller design.
[ Kf, m_h, m_w, m_f, m_b, Lh, La, Lw, g, K_EC_T, K_EC_P, K_EC_E ] = setup_heli_3d_configuration();
%
% For the following state vector: X = [ elevation; pitch; travel, elev_dot, pitch_dot, travel_dot]
% Initialization the state-Space representation of the open-loop System
HELI3D_ABCD_eqns;
% Augment state: Xi = [ elevation; pitch; travel, elev_dot, pitch_dot, travel_dot, elev_int, travel_int]
Ai = A;
Ai(7,1) = 1; % elevation integrator 
Ai(8,3) = 1; % travel integrator 
Ai(8,8) = 0;
Bi = B;
Bi(8,2) = 0;
%

C=eye(6,6)
D=zeros(6,2)
sys = ss(A, B, C, D);




%% === Dataset y Entrenamiento (Directo: Estados+Setpoints -> Voltajes LQR) ===
%clearvars -except A B Ai Bi K sys;  % asume ya tienes A,B,Ai,Bi,K,sys en workspace
rng(0);

% ----- Parámetros de simulación -----
num_trajectories = 300;   % cantidad de trayectorias (ajusta según recursos)
T_traj = 30.0;             % segundos por trayectoria
freq=1/5;

Ts = freq             % paso de muestreo para registro (50 Hz)
t_vec = 0:Ts:T_traj;
Nsteps = length(t_vec);

% límites de voltaje (satura durante simulación)
u_max = 7;
u_min = -7;

% ruido en mediciones (usar durante registro para robustez)
sigma_state = [0.002, 0.002, 0.002, 0.01, 0.01, 0.01]; % ruido por componente (ej.)

% Prealocar (estimación)
N_total = num_trajectories * Nsteps;
X = zeros(N_total, 9);  % 6 estados + 3 setpoints
Y = zeros(N_total, 2);  % 2 voltajes

idx = 1;

fprintf('Generando dataset usando LQR ...\n');
for traj = 1:num_trajectories
    % --- Generar setpoint time-varying simple pero variado ---
    % Puedes mezclar senos y escalones por trayectoria
    sp_type = mod(traj,4);
    switch sp_type
        case 0  % escalones aleatorios
            elev_sp = (rand()*1.0-0.5) * ones(size(t_vec));   % rad
            pitch_sp = (rand()*0.6-0.3) * ones(size(t_vec));
            travel_sp = (rand()*1.0-0.5) * ones(size(t_vec));
        case 1  % senoidales
            elev_sp = 0.5*sin(2*pi*(0.1+0.2*rand())*t_vec) + (rand()*0.2-0.1);
            pitch_sp = 0.2*sin(2*pi*(0.2+0.4*rand())*t_vec + rand()*pi);
            travel_sp = 0.4*sin(2*pi*(0.05+0.1*rand())*t_vec);
        case 2  % rampas
            elev_sp = linspace((rand()*0.6-0.3),(rand()*0.6-0.3),Nsteps);
            pitch_sp = linspace((rand()*0.4-0.2),(rand()*0.4-0.2),Nsteps);
            travel_sp = linspace((rand()*0.8-0.4),(rand()*0.8-0.4),Nsteps);
        otherwise % ruido alrededor de cero
            elev_sp = 0.05*randn(size(t_vec));
            pitch_sp = 0.03*randn(size(t_vec));
            travel_sp = 0.05*randn(size(t_vec));
    end

    % Estado inicial aleatorio pequeño (6 estados)
    %x0 = 0.02 * randn(6,1);
    x0 = zeros(6,1);
    
    % Integradores iniciales (para x_aug)
    z0 = [0; 0]; % elev_int, travel_int (tal como construiste Ai)
    xa0 = [x0; z0]; % estado aumentado 8x1

    % Simular con integración propia porque u = -K*(xa - xa_ref) depende de estado
    xa = xa0;
    for k = 1:Nsteps
        t_k = t_vec(k);
        % referencia instantánea (solo ángulos deseados; derivadas = 0)
        r = [elev_sp(k); pitch_sp(k); travel_sp(k)];

        % construir xa_ref (8x1) -> queremos que los primeros 6 componentes
        % contengan angles = [elev_d; pitch_d; travel_d; zeros(3) for derivatives]
        xa_ref = zeros(size(xa)); % por defecto 0
        xa_ref(1) = r(1);  % desired elevation
        xa_ref(2) = r(2);  % desired pitch
        xa_ref(3) = r(3);  % desired travel
        % derivadas y variables integrales en ref se mantienen 0 (ok)

        % Ley de control LQR (usando Ai,Bi,K calculado)
        % NOTA: K dimension es 2 x 8 porque usaste Ai,Bi
        u_star = -K * (xa - xa_ref); % 2x1

        % Saturación por seguridad
        u_star = min(max(u_star, u_min), u_max);

        % Registrar muestra con RUIDO de medición en los estados (simula sensor noise)
        x_meas = xa(1:6) + sigma_state'.*randn(6,1);

        % Construir entrada X y salida Y
        X(idx, :) = [x_meas' r'];
        Y(idx, :) = u_star';

        idx = idx + 1;

        % Integrar un paso de la planta REAL (6-states) y actualizar integradores:
        % dx = A*x + B*u  (estado real)
        dx = A * xa(1:6) + B * u_star;
        xa(1:6) = xa(1:6) + dx * Ts;

        % integradores: elev_int_dot = (elev_ref - elev_meas), travel_int_dot = (travel_ref - travel_meas)
        elev_err = xa_ref(1) - xa(1);   % desired - current elevation
        travel_err = xa_ref(3) - xa(3);
        xa(7) = xa(7) + elev_err * Ts;
        xa(8) = xa(8) + travel_err * Ts;

        % (opcional) añadir pequeña perturbación aleatoria a la dinámica para robustez
        xa(1:6) = xa(1:6) + 1e-4 * randn(6,1);

        % Si llegamos al final real del buffer, salimos
        if idx > N_total
            break;
        end
    end

    if mod(traj,50) == 0
        fprintf('Trayectoria %d/%d  (muestras registradas %d)\n', traj, num_trajectories, idx-1);
    end
end

% Recorte por si quedó espacio sin llenar
X = X(1:idx-1, :);
Y = Y(1:idx-1, :);

fprintf('Total muestras generadas: %d\n', size(X,1));

% ----- Normalización (z-score) -----
muX = mean(X,1); sigX = std(X,[],1) + 1e-9;
muY = mean(Y,1); sigY = std(Y,[],1) + 1e-9;

Xn = (X - muX) ./ sigX;
Yn = (Y - muY) ./ sigY;

% ----- Split train/val/test -----
N = size(Xn,1);
idx_perm = randperm(N);
Ntrain = round(0.75*N);
Nval = round(0.10*N);
train_idx = idx_perm(1:Ntrain);
val_idx = idx_perm(Ntrain+1:Ntrain+Nval);
test_idx = idx_perm(Ntrain+Nval+1:end);

Xtrain = Xn(train_idx,:);
Ytrain = Yn(train_idx,:);
Xval = Xn(val_idx,:);
Yval = Yn(val_idx,:);
Xtest = Xn(test_idx,:);
Ytest = Yn(test_idx,:);

% ----- Entrenamiento de la NN (9->2) -----
fprintf('Entrenando red neuronal 9->2 ...\n');

% usaremos Deep Learning Toolbox (trainNetwork)
layers = [
    featureInputLayer(9,"Name","input")
    fullyConnectedLayer(256,"Name","fc1")
    tanhLayer("Name","tanh1")
    fullyConnectedLayer(256,"Name","fc2")
    tanhLayer("Name","tanh2")
    fullyConnectedLayer(64,"Name","fc3")
    tanhLayer("Name","tanh3")
    fullyConnectedLayer(2,"Name","fc_out")
    regressionLayer("Name","reg")
];

opts = trainingOptions('adam', ...
    'MaxEpochs',40, ...
    'MiniBatchSize',256, ...
    'InitialLearnRate',1e-3, ...
    'Shuffle','every-epoch', ...
    'Plots','None', ...
    'Verbose',true, ...
    'ValidationData',{Xval, Yval}, ...
    'ValidationFrequency',1/freq, ...
    'ExecutionEnvironment','auto');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Mdl = trainNetwork(Xtrain, Ytrain, layers, opts);

% ----- Evaluación -----
Yhat_test = predict(Mdl, Xtest);
% des-normalizar
Yhat_test_den = Yhat_test .* sigY + muY;
Ytest_den = Ytest .* sigY + muY;

mse_total = mean((Yhat_test_den - Ytest_den).^2,'all');
mse_v1 = mean((Yhat_test_den(:,1) - Ytest_den(:,1)).^2);
mse_v2 = mean((Yhat_test_den(:,2) - Ytest_den(:,2)).^2);

fprintf('Test MSE total: %.6f | V1: %.6f | V2: %.6f\n', mse_total, mse_v1, mse_v2);

% ----- Guardado de modelo y normalizadores -----
helicopter_models.Mdl = Mdl;
helicopter_models.muX = muX; helicopter_models.sigX = sigX;
helicopter_models.muY = muY; helicopter_models.sigY = sigY;
helicopter_models.K = K;
save('helicopter_direct_controller.mat','helicopter_models','-v7.3');

fprintf('Modelo entrenado guardado en helicopter_direct_controller.mat\n');

% ----- Ejemplo de inferencia (uso en Simulink/MATLAB) -----
% Para predecir: X_in = [x(1:6)' , setpoint(1:3)'];  Xn = (X_in - muX) ./ sigX;
% u_norm = predict(Mdl, Xn); u = u_norm .* sigY + muY;




