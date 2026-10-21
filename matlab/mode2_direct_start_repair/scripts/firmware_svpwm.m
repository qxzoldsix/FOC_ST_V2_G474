function [voltage,ccr]=firmware_svpwm(command,iabc,q)
% Original sector logic and CCR truncation/clamp; reconstruct average voltages.
a=command(1); b=command(2);
t1=b; t2=b*0.5+a*0.8660254; t3=t2-t1;
sector=3;
if t2>0, sector=sector-1; end
if t3>0, sector=sector-1; end
if t1<0, sector=7-sector; end
if sector==1 || sector==4
    t=[t2;t1-t3;-t2];
elseif sector==2 || sector==5
    t=[t3+t2;t1;-t1];
elseif sector==3 || sector==6
    t=[t3;-t3;-(t1+t2)];
else
    t=zeros(3,1);
end
km=1;
if q.Vbus>1, km=min(max(12/q.Vbus,0.7),1.3); end
ccr=min(max(fix(t*km*2000)+2000,0),3999);
phase=ccr/4000*q.Vbus;
% Plant deadtime is an explicit physical assumption, independently configurable.
physical_i=iabc/q.current_channel_sign;
sgn=double(physical_i>0.01)-double(physical_i< -0.01);
phase=phase-q.actual_deadtime_V*sgn;
voltage=[(2*phase(1)-phase(2)-phase(3))/3;(phase(2)-phase(3))/sqrt(3)];
end
