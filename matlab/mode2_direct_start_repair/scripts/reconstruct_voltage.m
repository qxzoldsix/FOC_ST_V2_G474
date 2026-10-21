function voltage=reconstruct_voltage(old_command,old_ab,q)
% Known PWM command + previous measured current; no physical plant feedback.
old_phase=[old_ab(1);-0.5*old_ab(1)+sqrt(3)/2*old_ab(2);-0.5*old_ab(1)-sqrt(3)/2*old_ab(2)];
observer=q; observer.actual_deadtime_V=q.observer_deadtime_V;
[voltage,~]=firmware_svpwm(old_command,old_phase,observer);
end
