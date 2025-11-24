% On WinCon Controller Exit, set all the setpoint to zero, i.e. set the
% slider gains to 0.
% Tower Slider Gain Setpoint
set_param('q_3d_crane/Setpoints/Tower Setpoint (deg)','Gain','0');
% Trolley Slider Gain Setpoint
set_param('q_3d_crane/Setpoints/Trolley Setpoint (m)','Gain','0');
% Payload Slider Gain Setpoint
set_param('q_3d_crane/Setpoints/Payload Setpoint (m)','Gain','0');