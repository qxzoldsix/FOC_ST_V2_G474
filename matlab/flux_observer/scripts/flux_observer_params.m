function p = flux_observer_params(precision)
% Parameters copied from origin/环路参数辨识, c43e794924c73d0aec71ddcb6ee14937aa39835e.
% flux.h / Flux_Observer_Init / Deadtime_Comp_ab / ADC_SAMPLE.h.
p.source_commit = 'c43e794924c73d0aec71ddcb6ee14937aa39835e';
p.Rs = 0.875;                 % ohm
p.Ls = 0.0002; p.Ld = 0.0002; p.Lq = 0.0002; % H
p.Flux = 0.00752;             % Wb
p.MOTOR_POLES = 14;           % Keep firmware value and its RPM formula.
p.Ts = 5e-5;                 % 20 kHz
p.Gain = 5000;
p.PLL_BW_Hz = 15;
p.Comp_BW_Hz = 5;
p.PLL_kp = 2 * (2*pi*p.PLL_BW_Hz);
p.PLL_ki = (2*pi*p.PLL_BW_Hz)^2 * p.Ts; % ALREADY includes Ts
p.Comp_Kp = 2 * (2*pi*p.Comp_BW_Hz);
p.Comp_Ki = (2*pi*p.Comp_BW_Hz)^2 * p.Ts; % ALREADY includes Ts
p.Comp_Mode = 0;             % 0: PI (firmware default); 1: SMO feedback
p.SMO_Kslide = 10; p.SMO_Boundary = 0.002;
p.V_deadtime = 0.3;
p.current_deadband = 0.01;
p.nominal_bus_voltage = 12;
p.pll_step_limit = 2*pi*500*p.Ts;
p.speed_decimation = 8;
p.speed_filter_old = 0.95;
p.speed_filter_new = 0.05;
if nargin>0 && strcmp(precision,'single')
    fields = fieldnames(p);
    for k=1:numel(fields)
        if isnumeric(p.(fields{k})), p.(fields{k}) = single(p.(fields{k})); end
    end
    % Match each original C float operation, including intermediate rounding.
    pi_f = single(pi);
    w = 2*pi_f*p.PLL_BW_Hz;
    p.PLL_kp = 2*w; p.PLL_ki = w*w*p.Ts;
    w = 2*pi_f*p.Comp_BW_Hz;
    p.Comp_Kp = 2*w; p.Comp_Ki = w*w*p.Ts;
    p.pll_step_limit = 2*pi_f*500*p.Ts;
end
end
