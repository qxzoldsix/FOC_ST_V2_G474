function applied_command=deadtime_feedforward(desired_command,phase_i,estimated_rpm,time,q)
% Inverter voltage feedforward using measured phase-current polarity.
% This compensates physical voltage loss, not the displayed torque signal.
km=min(max(12/q.Vbus,0.7),1.3);
polarity=double(phase_i>0.01)-double(phase_i< -0.01);
time_gate=min(max((time-q.feedforward_start_s)/q.feedforward_ramp_s,0),1);
speed_gate=min(max((abs(estimated_rpm)-80)/80,0),1);
loss=time_gate*speed_gate*q.deadtime_feedforward_V*[(2*polarity(1)-polarity(2)-polarity(3))/3; ...
    (polarity(2)-polarity(3))/sqrt(3)];
applied_command=desired_command+sqrt(3)/(q.Vbus*km)*loss;
end
