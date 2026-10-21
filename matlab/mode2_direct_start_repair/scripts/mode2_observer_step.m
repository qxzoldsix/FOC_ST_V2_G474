function [y,s] = mode2_observer_step(u,s,p)
% Hybrid observer equations, original PI gains and original speed derivation.
% Repair frame = [applied volts(2); phase currents(3); fresh Clarke(2); Id; Vbus; Km].
u = u(:);
v = u(1:2); % Reconstructed applied VOLTS; deadtime compensation is upstream.
i = u(6:7);
cs = [cos(s.theta);sin(s.theta)];
psi_i = (p.Flux+(p.Ld-p.Lq)*u(8))*cs;
psi_v = s.psi_s-p.Lq*i; % BEFORE updating the stator integral
err = psi_i-psi_v;
if p.Comp_Mode == 0
    s.comp_integral = s.comp_integral+p.Comp_Ki*err;
    comp = p.Comp_Kp*err+s.comp_integral; % updated integral
else
    comp = p.SMO_Kslide*min(max(err/p.SMO_Boundary,-1),1);
end
s.psi_s = s.psi_s+p.Ts*(v-p.Rs*i+comp);
pll_err = -psi_v(1)*cs(2)+psi_v(2)*cs(1);
s.pll_integral = min(max(s.pll_integral+s.pll_ki*pll_err, ...
    -p.pll_step_limit),p.pll_step_limit);
step = min(max(s.pll_kp*pll_err+s.pll_integral, ...
    -p.pll_step_limit),p.pll_step_limit);
s.theta = mod(s.theta+step,2*pi); % PLL output is rad/sample, NOT rad/s
s.speed_acc = s.speed_acc+step;
s.speed_count = s.speed_count+1;
if s.speed_count >= p.speed_decimation
    hz = s.speed_acc/(p.speed_decimation*p.Ts*2*pi);
    s.speed_hz = p.speed_filter_old*s.speed_hz+p.speed_filter_new*hz;
    s.speed_acc = 0; s.speed_count = 0;
end
if s.pll_ramp_active
    s.pll_kp = s.pll_kp+0.02*(p.PLL_kp-s.pll_kp);
    s.pll_ki = s.pll_ki+0.02*(p.PLL_ki-s.pll_ki);
    if abs(s.pll_kp-p.PLL_kp)<0.05 && abs(s.pll_ki-p.PLL_ki)<0.05
        s.pll_kp = p.PLL_kp; s.pll_ki = p.PLL_ki;
        s.pll_ramp_active = false;
    end
end
% [theta(rad), electrical Hz, firmware RPM, active flux ab(Wb), stator
% flux ab(Wb), PLL error(Wb), step(rad/sample), corrected voltage ab(V)]
y = [s.theta;s.speed_hz;s.speed_hz*60/(p.MOTOR_POLES/2); ...
     psi_v;s.psi_s;pll_err;step;v];
end


