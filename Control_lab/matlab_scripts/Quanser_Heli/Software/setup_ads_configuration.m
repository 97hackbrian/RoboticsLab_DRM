%% SETUP_ADS_CONFIGURATION
%
% Returns various parameters for the Active Disturbance System on the
% Quanser 3-DOF Helicopter system.
%
% Copyright (C) 2007 Quanser Consulting Inc.
% Quanser Consulting Inc.
% 
%% Start of setup_ads_configuration()
function [ K_ENC, POS_HOME, CAL_VEL, CAL_TIME, wcf, kp, X_MAX, V_MAX  ] = setup_ads_configuration( )
    % Calculate Useful Conversion Factors
    [ K_R2D, K_D2R, K_IN2M, K_M2IN, K_RDPS2RPM, K_RPM2RDPS, K_OZ2N, K_N2OZ, K_LBS2N, K_N2LBS, K_G2MS, K_MS2G ] = calc_conversion_constants();
    % Armature Resistance (Ohm)
    Rm = 2.6;
    % Motor Torque Constant (N.m/A)
    Kt = 1.088 * K_OZ2N * K_IN2M; % = .00767
    % Motor Back-EMF Constant (V.s/rd)
    Km = 0.804e-3 * K_RDPS2RPM; % = .00767
    % Total gear ratio
    Kg = 3.71;
    % Ball-screw pitch (in/rev)
    Pb = 1/3;
    % Linear position encoder calibration gain (m/count)
    K_ENC = - 0.0254 * Pb / 4096;
    % Home or middle position (m)
    POS_HOME = 11 * 0.0254 / 2;
    % Calibration threshold velocity (m/s)
    CAL_VEL = 0.02;
    % Initial calibration time (s)
    CAL_TIME = 1.0;
    % Specifications of a second-order low-pass filter
    wcf = 2 * pi * 2.5; % filter cutting frequency
    % Proportional control gain (V/m)
    kp = 225;
    % Maximum travel limit (m)
    X_MAX = 11 * 0.024;
    % Velocity limit (m/s)
    V_MAX = 0.08;
    %
end