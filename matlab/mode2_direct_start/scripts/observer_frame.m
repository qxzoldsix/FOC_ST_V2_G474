function frame=observer_frame(old_command,phase_i,old_ab,old_dq,q)
% Angel_Get precedes UVW_Axis_DQ: fresh phase current, previous Clarke/Park.
km=1;
if q.Vbus>1, km=min(max(12/q.Vbus,0.7),1.3); end
frame=[old_command;phase_i;old_ab;old_dq(1);q.Vbus;km];
end
