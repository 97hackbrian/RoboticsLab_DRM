% CONFIG_3D_CRANE
%
% Sets up the 3-DOF Crane Rev 4.0 Anti-Backlash system parameters and
% encoder calibration constants.
%
% Summary of system nomenclature:
% g             Gravitational acceleration                          (m/s^2) 
% Kt_t          Motor Torque Constant: Tower                        (N.m/A)
% Kt_j          Motor Torque Constant: Jib                          (N.m/A)
% Kt_y          Motor Torque Constant: Trolley                      (N.m/A)
% eff_m_t       Tower Motor Efficiency
% eff_m_j       Jib Motor Efficiency
% eff_m_y       Trolley Motor Efficiency
% Kg_t          Motor Gear Ratio: Tower
% Kg_j          Motor Gear Ratio: Jib
% Kg_y          Motor Gear Ratio: Trolley
% eff_g_t       Gearbox Efficiency: 
% eff_g_j       Gearbox Efficiency: 
% eff_g_y       Gearbox Efficiency: 
% J_theta       Equivalent Moment of Inertia about Tower Pivot      (kg.m^2)
% J_psi         Equivalent Moment of Inertia about Trolley Pivot    (kg.m^2)
% J_phi         Equivalent Moment of Inertia about Payload Pivot    (kg.m^2)
% J_alpha       Equivalent Moment of Inertia about Alpha Pendulum Pivot  (kg.m^2)
% J_gamma       Equivalent Moment of Inertia about Gamma Pendulum Pivot  (kg.m^2)
% lj            Max distance of trolley from pivot                  (m)
% lp            Vertical distance of payload from jib arm           (m)
% mp            Mass of the payload                                 (kg)
% m_trolley     Mass of trolley                                     (kg)
% r_y_reel      Radius of trolley reel mounted on trolley motor
%               (used to lift and release payload position)         (m)
% r_j_pulley    Radius of pulley wheel mount on jib motor 
%               (used to move around the trolley)                   (m)
% Ka            AMPAQ gain                                          (A/V)
% K_CURR        AMPAQ current sensor calibration constnat           (A/V)
% K_ENC_THETA   Tower encoder calibration constant                  (rad/count)
% K_ENC_X       Trolley position encoder calibration constant       (m/count)
% K_ENC_Z       Payload elevation encoder calibration constant      (m/count)
% K_ENC_ALPHA   Gimble alpha deflection calibration constant        (rad/count)
% K_ENC_GAMMA   Gimble gamma deflection calibration constant       (rad/count)
%
% Copyright (C) 2010 Quanser Consulting Inc.
% Quanser Consulting Inc.
%
function [g, Kt_t, Kt_j, Kt_y, eff_m_t, eff_m_j, eff_m_y, Kg_t, Kg_j, Kg_y, eff_g_t, eff_g_j, eff_g_y, J_theta, J_psi, J_phi, J_alpha, J_gamma, lj, lp, mp, m_trolley, r_y_reel, r_j_pulley, Ka, K_CURR, K_ENC_THETA, K_ENC_X, K_ENC_Z, K_ENC_ALPHA, K_ENC_GAMMA ] = config_3d_crane()
% Conversion factors
[ K_R2D, K_D2R, K_IN2M, K_M2IN, K_RDPS2RPM, K_RPM2RDPS, K_OZ2N, K_N2OZ, K_LBS2N, K_N2LBS, K_G2MS, K_MS2G ] = calc_conversion_constants ();
% Gravitational constant (m/s^2)
g = 9.81;
% Pound to kg
% K_LBS2KG = K_LBS2N / g;
%
%% Tower Motor Parameters
% Motor current-torque constant (N.m/A)
Kt_t = 8.93;
% Motor efficiency
eff_m_t = 1.0;
% Motor Gearbox ratio
Kg_t = 100;
% Motor Gearbox efficiency
eff_g_t = 1.0;
% Motor Rotor Moment Of Inertia (kg.m^2)
Jm = 1.02e-6;
% Equivalent Rotor Moment Of Inertia at the harmonic drive shaft output (kg.m^2)
Jm_t = Kg_t^2 * Jm; % = 0.0102
%
%% Cart Motor Parameters
% Faulhaber 3042W 024C
% i.e. the motor that actuates the trolley movement
% Current-torque constant (N.m/A)
Kt_j = 5.608 * K_IN2M * K_OZ2N; % = 0.0396;
% Motor efficiency
eff_m_j = 0.79;
% Motor gearbox ratio
Kg_j = 3.7;
% Motor gearbox efficiency
eff_g_j = 0.95;
% Motor armature moment of inertia (kg.m^2)
Jm_j = 2.266e-4 * K_OZ2N / g * K_IN2M; % 1.6311e-007
%
%% Cart Linear Guide
% Pitch (m/rev)
Pt = 0.5 * K_IN2M;
%
%% Payload Motor Parameters
% Faulhaber 2342S 024CR
% Current-torque constant (N.m/A)
Kt_y = 0.0261;
% Motor efficiency
eff_m_y = 0.81;
% Motor gearbox (internal gear ratio)
Kg_y_i = 14;
% Motor to pulley gear ratio (external)
Kg_y_e = 2;
% Equivalent gear ratio from motor to pulley
Kg_y = Kg_y_i*Kg_y_e;
% Motor gearbox efficiency
eff_g_y = 1;
% Motor armature moment of inertia (kg.m^2)
Jm_y = 5.8/1000/100^2;
%
%% Trolley-related Mass and Length Parameters
% Payload mass (kg)
mp = 0.147; % mass of brass weight, black cover, and hook
% Maximum distance of payload from trolley (m)
lp = 34 * K_IN2M;
% Trolley mass (kg)
m_trolley = 0.60; % estimated using CAD
% Maximum position of trolley (m)
lj = 31.75 * K_IN2M;
%
%% Moment of Inertia about Tower Pivot
% Total mass of jib structure (kg)
m_j = 4.08;
% Total length of jib (m)
L_j = 1.205;
% Length of arm from tower pivot to end of trolley side (m)
L_j1 = 35.75 * K_IN2M; 
% Length of jib from tower pivot to back end (m)
L_j2 = 12 * K_IN2M; 
% Effective mass of front end (kg)
m_j1 = L_j1 * m_j / L_j;
% Effective mass of back end (kg)
m_j2 = L_j2 * m_j / L_j;
% Moment of inertia acting on tower pivot from jib (kg.m^2)
J_t_jib = 1 / 3 * m_j1 * L_j1^2 + 1/3 * m_j2 * L_j2^2; 
%
%% Moment of Inertia of Jib
% This is the moment of inertia about the motor that moves the cart.
% Radius of jib motor pulley (m)
r_j_pulley = 0.560 / 2 * K_IN2M;
% Radius of jib motor pulley to outside rim (m)
r_j_pulley_od = 0.740 / 2 * K_IN2M;
% Mass of jib motor pulley (m)
m_j_pulley = 0.017; % **** MEASURE!!!
% Moment of inertia from pulley (kg.m^2)
J_j_pulley = 1/2 * m_j_pulley * r_j_pulley_od^2;
%
%% Moment of Inertia from Reel Mounted on Trolley Motor Shaft
% Radius of trolley reel (m)
r_y_reel = 1.0 / 2 * K_IN2M;
% Mass of trolley reel (kg)
m_y_reel = 0.015;
% Moment of inertia of trolley reel (kg.m^2)
J_y_reel = 1/2 * m_y_reel * r_y_reel^2;
%
%% Equivalent Moment of Inertia Calculations
% Tower equivalent moment of inertia (kg.m^2)
J_theta = Jm_t + J_t_jib;
% Jib equivalent moment of inertia (kg.m^2)
J_psi = Jm_j + J_j_pulley;
% Trolley equivalent moment of inertia (kg.m^2)
J_phi = Jm_y + J_y_reel;
% Gimble alpha deflection equivalent moment of inertia (kg.m^2)
J_alpha = 0;
% Gimble gamma deflection equivalent moment of inertia (kg.m^2)
J_gamma = 0;
%
%% AMPAQ Specifications
% Gain (A/V)
Ka = 0.5;
% Current sensor calibration consant (A/V)
K_CURR = -2.0;
%
%% Encoder Calibration Gains
% Tower encoder calibration constant (rad/count)
K_ENC_THETA = 2 * pi / (4 * 1024 ) / Kg_t;
% Trolley position encoder calibration constant (m/count)
K_ENC_X = - Pt / (4 * 1024 ) / Kg_j;
% Payload elevation encoder calibration constant (m/count)
K_ENC_Z = - 2 * pi * r_y_reel / (4 * 1024 ) / Kg_y;
% Gimble alpha deflection calibration constant (rad/count)
K_ENC_ALPHA = 2 * pi / (4 * 1024 );
% Gimble gamma deflection (calibration constant rad/count)
K_ENC_GAMMA = 2 * pi / (4 * 1024 );