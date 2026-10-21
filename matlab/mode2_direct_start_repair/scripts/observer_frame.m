function frame=observer_frame(old_command,phase_i,old_ab,old_dq,q)
% Reconstruct APPLIED voltage from known previous PWM command/CCR and
% measured current polarity. No physical angle, speed or plant voltage input.
km=1;
if q.Vbus>1, km=min(max(12/q.Vbus,0.7),1.3); end
v=reconstruct_voltage(old_command,old_ab,q);
ab=[phase_i(1);(phase_i(1)+2*phase_i(2))*0.577350269];
frame=[v;phase_i;ab;old_dq(1);q.Vbus;km];
end
