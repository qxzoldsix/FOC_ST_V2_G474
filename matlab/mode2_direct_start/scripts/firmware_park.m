function [dq,iab,cs]=firmware_park(iabc,theta,fSin)
% Foc.c CURRENT_USE_W_AS_V=0, CURRENT_BETA_SIGN=1, PARK_ANGLE_REVERSE=0.
iab=[iabc(1);(iabc(1)+2*iabc(2))*0.577350269];
angle=fix(651.8986*theta);
if angle>4095, angle=angle-4096; elseif angle<0, angle=angle+4096; end
angle=mod(angle,4096); sector=floor(angle/1024);
% Preserve the original quarter-wave table indices, including 0x3FF offsets.
switch sector
    case 0
        s=fSin(angle+1); c=fSin(1023-angle+1);
    case 1
        s=fSin(2048-angle+1); c=-fSin(angle-1023+1);
    case 2
        s=-fSin(angle-2048+1); c=-fSin(3072-angle+1);
    otherwise
        s=-fSin(4096-angle+1); c=fSin(angle-3072+1);
end
cs=[c;s]; dq=[iab(1)*c+iab(2)*s;iab(2)*c-iab(1)*s];
end
