% Matlab equation file: "CRANE_TOWER_ABCD_eqns.m"
% Open-Loop State-Space Matrices: A, B, C, and D
% for the Quanser 3D CRANE Experiment.

A( 1, 1 ) = 0;
A( 1, 2 ) = 0;
A( 1, 3 ) = 1;
A( 1, 4 ) = 0;
A( 2, 1 ) = 0;
A( 2, 2 ) = 0;
A( 2, 3 ) = 0;
A( 2, 4 ) = 1;
A( 3, 1 ) = 0;
A( 3, 2 ) = mp^2*lp^2*lj*g/(J_alpha*J_theta+J_theta*mp*lp^2+mp*lj^2*J_alpha);
A( 3, 3 ) = 0;
A( 3, 4 ) = 0;
A( 4, 1 ) = 0;
A( 4, 2 ) = -mp*lp*g*(mp*lj^2+J_theta)/(J_alpha*J_theta+J_theta*mp*lp^2+mp*lj^2*J_alpha);
A( 4, 3 ) = 0;
A( 4, 4 ) = 0;

B( 1, 1 ) = 0;
B( 2, 1 ) = 0;
B( 3, 1 ) = eff_g_t*Kg_t*eff_m_t*Kt_t*(J_alpha+mp*lp^2)/(J_alpha*J_theta+J_theta*mp*lp^2+mp*lj^2*J_alpha);
B( 4, 1 ) = -mp*lp*eff_g_t*Kg_t*eff_m_t*Kt_t*lj/(J_alpha*J_theta+J_theta*mp*lp^2+mp*lj^2*J_alpha);

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
