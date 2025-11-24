% Cargar modelos guardados
%clear; clc; close all;
%v1

load('helicopter_direct_controller.mat')

Mdl  = helicopter_models.Mdl
muX  = helicopter_models.muX
sigX = helicopter_models.sigX
muY  = helicopter_models.muY
sigY = helicopter_models.sigY

save('Model_helicopter_direct_controller.mat','Mdl','-v7.3');

A = sys.A;
B = sys.B;

Ts = 0.02;
T = 60;
t = 0:Ts:T;

% Setpoint fijo (ejemplo)
sp = [7.5; 0.0; 0.0];  % [elev_d, pitch_d, travel_d]

% Estado inicial
x = zeros(6,1);

Xlog = zeros(length(t),6);
Ulog = zeros(length(t),2);

for k = 1:length(t)

    % FORMAR ENTRADA DE LA NN
    X_in = [x' sp'];   % 1x9

    % NORMALIZAR ENTRADA
    Xn = (X_in - helicopter_models.muX) ./ helicopter_models.sigX;

    % NN → voltajes normalizados
    yn = predict(helicopter_models.Mdl, Xn);

    % DESNORMALIZAR
    u = yn .* helicopter_models.sigY + helicopter_models.muY;
    u = u(:);

    % GUARDAR LOGS
    Xlog(k,:) = x';
    Ulog(k,:) = u';

    % SIMULAR PLANTA LINEAL (Euler)
    dx = A*x + B*u;
    x = x + Ts*dx;
end

% Graficar elevación controlada
%figure; plot(t, Xlog(:,1), 'LineWidth',2); hold on;
%yline(sp(1),'--r','Setpoint');
%xlabel('tiempo'); ylabel('elevación');
%title('Control directo NN sobre modelo del helicóptero');
