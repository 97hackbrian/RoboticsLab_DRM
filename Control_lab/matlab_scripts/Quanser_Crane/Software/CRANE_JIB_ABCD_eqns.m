% Matlab equation file: "CRANE_JIB_ABCD_eqns.m"
% Open-Loop State-Space Matrices: A, B, C, and D
% for the Quanser 3D CRANE TROLLEY/PENDULUM MODEL Experiment.

A( 1, 1 ) = 0;
A( 1, 2 ) = 0;
A( 1, 3 ) = 1;
A( 1, 4 ) = 0;
A( 2, 1 ) = 0;
A( 2, 2 ) = 0;
A( 2, 3 ) = 0;
A( 2, 4 ) = 1;
A( 3, 1 ) = 0;
A( 3, 2 ) = -r_j_pulley^2*mp^2*lp^2*g/(m_trolley*r_j_pulley^2*mp*lp^2+m_trolley*r_j_pulley^2*J_gamma+mp*r_j_pulley^2*J_gamma+J_psi*Kg_j^2*mp*lp^2+J_gamma*Kg_j^2*J_psi);
A( 3, 3 ) = -r_j_pulley^2*B_psi*(mp*lp^2+J_gamma)/(m_trolley*r_j_pulley^2*mp*lp^2+m_trolley*r_j_pulley^2*J_gamma+mp*r_j_pulley^2*J_gamma+J_psi*Kg_j^2*mp*lp^2+J_gamma*Kg_j^2*J_psi);
A( 3, 4 ) = -r_j_pulley^2*mp*lp*B_gamma/(m_trolley*r_j_pulley^2*mp*lp^2+m_trolley*r_j_pulley^2*J_gamma+mp*r_j_pulley^2*J_gamma+J_psi*Kg_j^2*mp*lp^2+J_gamma*Kg_j^2*J_psi);
A( 4, 1 ) = 0;
A( 4, 2 ) = -mp*g*lp*(m_trolley*r_j_pulley^2+mp*r_j_pulley^2+J_psi*Kg_j^2)/(m_trolley*r_j_pulley^2*mp*lp^2+m_trolley*r_j_pulley^2*J_gamma+mp*r_j_pulley^2*J_gamma+J_psi*Kg_j^2*mp*lp^2+J_gamma*Kg_j^2*J_psi);
A( 4, 3 ) = -B_psi*r_j_pulley^2*mp*lp/(m_trolley*r_j_pulley^2*mp*lp^2+m_trolley*r_j_pulley^2*J_gamma+mp*r_j_pulley^2*J_gamma+J_psi*Kg_j^2*mp*lp^2+J_gamma*Kg_j^2*J_psi);
A( 4, 4 ) = -B_gamma*(m_trolley*r_j_pulley^2+mp*r_j_pulley^2+J_psi*Kg_j^2)/(m_trolley*r_j_pulley^2*mp*lp^2+m_trolley*r_j_pulley^2*J_gamma+mp*r_j_pulley^2*J_gamma+J_psi*Kg_j^2*mp*lp^2+J_gamma*Kg_j^2*J_psi);

B( 1, 1 ) = 0;
B( 2, 1 ) = 0;
B( 3, 1 ) = r_j_pulley*eff_g_j*Kg_j*eff_m_j*Kt_j*(mp*lp^2+J_gamma)/(m_trolley*r_j_pulley^2*mp*lp^2+m_trolley*r_j_pulley^2*J_gamma+mp*r_j_pulley^2*J_gamma+J_psi*Kg_j^2*mp*lp^2+J_gamma*Kg_j^2*J_psi);
B( 4, 1 ) = eff_g_j*Kg_j*eff_m_j*Kt_j*r_j_pulley*mp*lp/(m_trolley*r_j_pulley^2*mp*lp^2+m_trolley*r_j_pulley^2*J_gamma+mp*r_j_pulley^2*J_gamma+J_psi*Kg_j^2*mp*lp^2+J_gamma*Kg_j^2*J_psi);

C( 1, 1 ) = 1;
C( 1, 2 ) = 0;
C( 1, 3 ) = 0;
C( 1, 4 ) = 0;
C( 2, 1 ) = 0;
C( 2, 2 ) = 1;
C( 2, 3 ) = 0;
C( 2, 4 ) = 0;

D( 1, 1 ) = 0;
D( 2, 1 ) = 0;
