function [iabc,iab,rpm,theta,torque]=phase_measurements(x,q)
% Averaged physical PMSM measurements. True angle never enters controller.
c=cos(x(4)); s=sin(x(4));
iab=[x(1)*c-x(2)*s;x(1)*s+x(2)*c];
iabc=[iab(1);-0.5*iab(1)+sqrt(3)/2*iab(2);-0.5*iab(1)-sqrt(3)/2*iab(2)];
iabc=q.current_channel_sign*iabc;
rpm=x(3)*60/(2*pi); theta=mod(x(4),2*pi);
torque=1.5*q.pole_pairs*(q.Flux*x(2)+(q.Ld-q.Lq)*x(1)*x(2));
end
