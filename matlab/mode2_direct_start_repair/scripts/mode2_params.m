function [p,q,s0]=mode2_params()
% p: firmware constants. q: simulation conditions / mechanics, NOT identified.
p.Rs=0.875; p.Ls=0.0002; p.Ld=0.0002; p.Lq=0.0002;
p.Flux=0.00752; p.MOTOR_POLES=14; p.Ts=5e-5;
p.Gain=5000; p.PLL_BW_Hz=15; p.Comp_BW_Hz=5;
p.PLL_kp=2*(2*pi*p.PLL_BW_Hz);
p.PLL_ki=(2*pi*p.PLL_BW_Hz)^2*p.Ts;
p.Comp_Kp=2*(2*pi*p.Comp_BW_Hz);
p.Comp_Ki=(2*pi*p.Comp_BW_Hz)^2*p.Ts;
p.Comp_Mode=0; p.SMO_Kslide=10; p.SMO_Boundary=0.002;
p.V_deadtime=0.3; p.current_deadband=0.01;
p.pll_step_limit=2*pi*500*p.Ts;
p.speed_decimation=8; p.speed_filter_old=0.95; p.speed_filter_new=0.05;
s0.psi_s=[0;0]; s0.theta=0;
q.Ts=p.Ts; q.Rs=p.Rs; q.Ld=p.Ld; q.Lq=p.Lq; q.Flux=p.Flux;
q.pole_pairs=p.MOTOR_POLES/2;
q.target_rpm=200; q.second_rpm=400; q.third_rpm=1000;
q.switch_1=0.8; q.switch_2=1.6; q.Vbus=12; q.stop_time=4;
q.J=2e-5; q.B=1e-5; q.load_torque=0.005;
q.load_speed_scale_rad_s=1; % Passive resisting load; 0 selects fixed external torque.
q.random_initial_angle=true; % Draw a new physical angle on each normal simulation.
q.initial_theta_e=pi/6; q.initial_rpm=0; % Fixed-angle fallback / nominal regression.
q.actual_deadtime_V=0.3; q.observer_deadtime_V=0.3; q.speed_iq_sign=1;
q.deadtime_feedforward_V=0.3; % Estimated inverter voltage loss, calibrated assumption.
q.feedforward_start_s=0.5; q.feedforward_ramp_s=0.5;
q.speed_kp=0.01; q.speed_ki=0.0001;
q.current_kp=0.01; q.current_ki=0.00001; q.iq_limit=12; % Preserve the source PI output limit.
q.reference_filter_hz=2; % Two-pole command shaping BEFORE the source slew limiter.
q.reference_start_delay_s=0.15; % Observer warmup at zero command; no alignment current.
q.ramp_step=1000*q.Ts; q.current_channel_sign=1;
% Physical phase currents are positive into the winding. ADC hardware polarity
% is not given by the firmware's raw-count minus sign alone. Explicit assumption.
q.initial_state=[0;0;q.initial_rpm*2*pi/60;q.initial_theta_e];
end
