function [ kp, kv, ki ] = d_payload_piv_controller( Kt_y, eff_m_y, Kg_y, eff_g_y, J_phi, mp, r_y_reel, PO, tp, p0 )

% I) calculate the natural frequency, w0, and the damping ratio, zeta,
% from the given percentage overshoot and peak time parameters
% i) spec #1: maximum Percent Overshoot (PO)
% PO = 10 => zeta = 0.59
% and tp = 0.15 => wn = 26.00 rd/s
if ( PO > 0 )
    % using the *Hint provided in the lab, zeta_min is given by:
    zeta_min = abs( log( PO / 100 ) ) / sqrt( pi^2 + log( PO / 100)^2 );
    zeta = zeta_min;
else
    error( 'Error: Set Percentage Overshoot.' )
end
% ii) spec #2: tp - using the *Hint provided in the lab:
w0 = pi / ( tp * sqrt( 1 - zeta^2 ) );
%
% II) calculate the required PIV controller gains to meet the desired
% specifications using the control law: 
% Im = (kp+ki/s)*(z_des-z) - kv*z_dot as detailed in the Maple worksheet.
kp = w0*(2*zeta*p0*J_phi+w0*mp*r_y_reel^2+w0*J_phi+2*zeta*p0*mp*r_y_reel^2)/eff_g_y/Kg_y/eff_m_y/Kt_y/r_y_reel;
kv = (p0*J_phi+2*zeta*w0*mp*r_y_reel^2+2*zeta*w0*J_phi+p0*mp*r_y_reel^2)/eff_g_y/Kg_y/eff_m_y/Kt_y/r_y_reel;
ki = w0^2*p0*(mp*r_y_reel^2+J_phi)/eff_g_y/Kg_y/eff_m_y/Kt_y/r_y_reel;