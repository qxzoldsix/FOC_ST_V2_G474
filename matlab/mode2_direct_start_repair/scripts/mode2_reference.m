function y=mode2_reference(p,q,fSin)
% Independent time-domain reference for the native block interconnection.
% Observer equations copied from flux.c; the PIs retain previous-v1 freezing.
n=round(q.stop_time/p.Ts)+1; y=zeros(n,34);
x=q.initial_state; cmd=[0;0]; ab=[0;0]; dq=[0;0];
s.psi_s=[0;0]; s.comp_integral=[0;0]; s.theta=0; s.pll_integral=0;
s.speed_acc=0; s.speed_count=0; s.speed_hz=0;
s.pll_kp=p.PLL_kp; s.pll_ki=p.PLL_ki; s.pll_ramp_active=false;
speed=zeros(3,1); d=speed; iq=speed; ramp=0; z1=0; z2=0;
for k=1:n
    t=(k-1)*p.Ts;
    target=q.target_rpm;
    if t>=q.switch_1, target=q.second_rpm; end
    if t>=q.switch_2, target=q.third_rpm; end
    [phase,~,rpm,theta,torque]=phase_measurements(x,q);
    frame=observer_frame(cmd,phase,ab,dq,q);
    [obs,s]=mode2_observer_step(frame,s,p);
    [dq,ab,cs]=firmware_park(phase,obs(1),fSin);
    reference_target=target;
    if isfield(q,'reference_start_delay_s') && t<q.reference_start_delay_s
        reference_target=0;
    end
    shaped=reference_target;
    if isfield(q,'reference_filter_hz') && q.reference_filter_hz>0
        alpha=1-exp(-2*pi*q.reference_filter_hz*p.Ts);
        z1=z1+alpha*(reference_target-z1); z2=z2+alpha*(z1-z2); shaped=z2;
    end
    ramp=ramp+min(max(shaped-ramp,-q.ramp_step),q.ramp_step);
    speed=pi_step(ramp-obs(3),speed,q.speed_kp,q.speed_ki,q.iq_limit);
    iqref=q.speed_iq_sign*speed(3);
    lim=min(max(q.Vbus/12,0),0.95);
    d=pi_step(-dq(1),d,q.current_kp,q.current_ki,lim);
    qlim=sqrt(max(lim^2-d(3)^2,0));
    iq=pi_step(iqref-dq(2),iq,q.current_kp,q.current_ki,qlim);
    vdq=[d(3);iq(3)]; cmd=firmware_inverse_park(vdq,cs);
    if isfield(q,'deadtime_feedforward_V'), cmd=deadtime_feedforward(cmd,phase,obs(3),t,q); end
    [v,ccr]=firmware_svpwm(cmd,phase,q);
    y(k,:)=[obs;rpm;theta;torque;phase;dq;vdq;iqref;ramp;cmd;v;ccr;target;2;ab].';
    x=pmsm_rk4(x,v,q);
end
end
function s=pi_step(error,s,kp,ki,limit)
if ~(s(2)>=limit || s(2)<=-limit), s(1)=s(1)+ki*error; end
s(1)=min(max(s(1),-limit),limit);
s(2)=kp*error+s(1);
out=min(max(s(2),-limit),limit);
s(3)=0.9*s(3)+0.1*out;
end
