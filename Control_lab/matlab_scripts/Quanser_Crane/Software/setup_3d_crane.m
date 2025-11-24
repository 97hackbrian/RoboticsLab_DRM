%% SETUP_3D_CRANE
%
% Designs a state-feedback controller using the LQR technique for the
% Quanser 3D Crane Revision 4.0 device.
%
% Copyright (C) 2010 Quanser Consulting Inc.
% Quanser Consulting Inc.
%
% clear workspace
clear;
%
%% ##### LIMIT SWITCHES and SAFETY WATCHDOGS  #####
% Enable/Diable Limit Switches
LS_LIM_ENABLE = 1;
% LS_LIM_ENABLE = 0;
% Safety Watchdog: Trolley ON/OFF
X_LIM_ENABLE = 1;  
% X_LIM_ENABLE = 0;
% Safety Limits on the trolley displacement (m)
X_MAX = 0.4;
X_MIN = - X_MAX;
% Safety Watchdog: Payload ON/OFF
Z_LIM_ENABLE = 1;
% Z_LIM_ENABLE = 0;
% Safety Limits on the payload displacement (m)
Z_MAX = 0.7;
Z_MIN = -0.05;
% Safety Watchdog: Alpha Pendulum Angle ON/OFF
ALPHA_LIM_ENABLE = 1;
% ALPHA_LIM_ENABLE = 0;
% Safety Limits on the pendulum angle (deg)
ALPHA_MAX = 25;
ALPHA_MIN = - ALPHA_MAX;
% Safety Watchdog: Gamma Pendulum Angle ON/OFF
GAMMA_LIM_ENABLE = 1;
% GAMMA_LIM_ENABLE = 0;
% Safety Limits on the pendulum angle (deg)
GAMMA_MAX = 25;            % pendulum angle maximum safety position (deg)
GAMMA_MIN = - GAMMA_MAX;   % pendulum angle minimum safety position (deg)
%
%% ##### HIGH-PASS FILTER PARAMETERS #####
% Tower HPF Parameters
wf_theta = 2 * pi * 50;
% Jib HPF Parameters
wf_x = 2 * pi * 50;
% Payload position HPF Parameters
wf_z = 2 * pi * 50;
% Alpha Gimble deflection HPF Parameters
wf_alpha = 2 * pi * 50;
% Gamma Gimble deflection HPF Parameters
wf_gamma = 2 * pi * 2.0;
% ##### END OF USER-DEFINED FILTER PARAMETERS #####
%
%% ##### BUILD OPEN-LOOP MODEL #####
% AMPAQ Maximum Output Current (A)
IMAX_AMP = 7;
% Digital-to-Analog Maximum Voltage (V); for Q4/Q8 cards set to 10
VMAX_DAC = 10;
% load 3D Crane model parameters and sensor calibration constants
[g, Kt_t, Kt_j, Kt_y, eff_m_t, eff_m_j, eff_m_y, Kg_t, Kg_j, Kg_y, eff_g_t, eff_g_j, eff_g_y, J_theta, J_psi, J_phi, J_alpha, J_gamma, lj, lp, mp, m_trolley, r_y_reel, r_j_pulley, Ka, K_CURR, K_ENC_THETA, K_ENC_X, K_ENC_Z, K_ENC_ALPHA, K_ENC_GAMMA ] = config_3d_crane();
% state-space of the 3D Crane Tower subsystem.
CRANE_TOWER_ABCD_eqns;
A_T = A; B_T = B; C_T = C; D_T = D; 
clear A B C D;
% state-space of the 3D Crane Jib subsystem.
B_psi = 0;
B_gamma = 0;
CRANE_JIB_ABCD_eqns;
% add an integrator to the Jib system
A_J = A; B_J = B; C_J = C; D_J = D;
A_J(5,1) = 1; 
A_J(5,5) = 0;
B_J(5) = 0;
%
clear A B C D;
%
%% ##### USER_DEFINED CONTROL SYSTEM DESIGN #####
% Tower controller
Q_T = diag([2 1 0.25 0.25]);
R_T = 0.05;
K_T = lqr(A_T,B_T,Q_T,R_T);
%
% Jib controller
Q_J = diag([5 5 1 5 1]);
% Q_J = diag([75 5 5 15 5]);
Q_J = diag([5 5 1 5 1]);
R_J = 0.05;
K_J = lqr(A_J,B_J,Q_J,R_J);
%
% Payload position controller
% Percentage overshoot specification (%)
PO = 10;
% Peak time specification (s)
% tp = 0.05;
tp = 0.02;
% Zero location specifiation (rad/s)
p0 = 0.125;
% Calculate PIV gains
[ kp, kv, ki ] = d_payload_piv_controller( Kt_y, eff_m_y, Kg_y, eff_g_y, J_phi, mp, r_y_reel, PO, tp, p0 );
%
%% ##### DEBOUNCE PARAMETERS #####
% Debounce is trigerred when the average of the last 'dbnc_samples' is
% greater than the threshold.
dbnc_threshold = 0.8;
% Number of samples in input signal used in average calculation.
dbnc_samples = 0.01 / 1e-3; % duration (s) dividied by sample time (Hz)
%
%% ##### CALIBRATION PARAMETERS #####
% Desired HOME positions after encoders have been reset
% Tower Home Setpoint (deg)
sp_home_tower = 155;
% Trolley Home Setpoint (m)
sp_home_trolley = 0.25;
% Payload Home Setpoint (deg)
sp_home_payload = -0.7;
%
% Calibration Threshold: abs of position & velocity error has to be 
% less than these values
% Tower Calibration Position Threshold (rad)
calib_theta = 4.5*pi/180;
% Trolley Calibration Position Threshold (m)
calib_x = 2/100;
% Payload Calibration Position Threshold (m)
calib_z = 3/100;
% Tower Calibration Velocity Threshold (rad/s)
calib_theta_dot = 1/1000;
% Trolley Calibration Velocity Threshold (m/s)
calib_x_dot = 2/1000;
% Payload Calibration Velocity Threshold (m/s)
calib_z_dot = 5/1000;
% Position threshold that detects if tower is stuck at right limit
% If the tower is not close to setpoint and at zero velocity, then it is at
% the right limit switch.
calib_theta_r = 15.0*pi/180;
%
%% ##### DISPLAY #####
disp(' ');
disp('Tower Control Gain:');
disp( ['    K_T  = [' num2str(K_T,3) ' ]' ]);
disp(' ');
disp('Jib Control Gain:');
disp( ['    K_J  = [' num2str(K_J,3) ' ]' ]);
disp(' ');
disp('PIV Trolley Control gains:');
disp( ['    kp = ' num2str(kp,3) ' A/m']);
disp( ['    ki = ' num2str(ki,3)  ' A/m/s']);
disp( ['    kv = ' num2str(kv,3) ' A.s/m' ]);