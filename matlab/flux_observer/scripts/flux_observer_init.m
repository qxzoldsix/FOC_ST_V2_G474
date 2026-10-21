function s = flux_observer_init(p,mode,first_input,alignment_angle)
% Default: original all-zero firmware state. Optional aligned flux seeding
% requires an established alignment angle, not the simulated true angle.
if nargin<2, mode='source'; end
s.psi_s = zeros(2,1,'like',p.Rs);
s.comp_integral = zeros(2,1,'like',p.Rs);
z = zeros(1,'like',p.Rs);
s.theta = z; s.pll_integral = z;
s.speed_acc = z; s.speed_count = 0; s.speed_hz = z;
s.pll_kp = p.PLL_kp; s.pll_ki = p.PLL_ki;
s.pll_ramp_active = false;
switch lower(mode)
    case 'source'
        % Preserve exact Flux_Observer_Init() reset for source regression.
    case 'aligned'
        assert(nargin>=3 && numel(first_input)==10,'Aligned initialization needs the first measured input frame.');
        if nargin<4, alignment_angle=0; end
        u = cast(first_input(:),'like',p.Rs);
        s.theta = cast(alignment_angle,'like',p.Rs);
        active = p.Flux+(p.Ld-p.Lq)*u(8);
        s.psi_s = p.Lq*u(6:7)+active*[cos(s.theta);sin(s.theta)];
        % PLL integral and reported speed remain ZERO: no true speed preload.
    otherwise
        error('Startup mode must be source or aligned.');
end
end
