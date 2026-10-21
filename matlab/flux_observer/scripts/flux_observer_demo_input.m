function [input,truth] = flux_observer_demo_input(p)
% Analytic rotating PMSM trajectory, not a closed-loop mechanical plant.
% Generate voltage using the SAME forward-Euler difference as the observer.
% No rotor angle or true flux is passed into the observer.
t = (0:p.Ts:2).';
f = 25*ones(size(t)); f(t>=0.8) = 40; f(t>=1.4) = 15;
theta = [0;cumsum(2*pi*f(1:end-1)*p.Ts)];
id = 0.2*ones(size(t)); iq = ones(size(t));
iab = [id.*cos(theta)-iq.*sin(theta), id.*sin(theta)+iq.*cos(theta)];
iabc = [iab(:,1), -0.5*iab(:,1)+sqrt(3)/2*iab(:,2), ...
                    -0.5*iab(:,1)-sqrt(3)/2*iab(:,2)];
psi = [(p.Flux+p.Ld*id).*cos(theta)-p.Lq*iq.*sin(theta), ...
       (p.Flux+p.Ld*id).*sin(theta)+p.Lq*iq.*cos(theta)];
% Initial observer flux is selected independently (source/aligned); input
% generation does not feed true rotor angle or speed into the observer.
v = [diff(psi)/p.Ts; (psi(end,:)-psi(end-1,:))/p.Ts]+p.Rs*iab;
sgn = double(iabc>p.current_deadband)-double(iabc<-p.current_deadband);
dist = [0.6666667*p.V_deadtime*(sgn(:,1)-0.5*sgn(:,2)-0.5*sgn(:,3)), ...
        0.5773503*p.V_deadtime*(sgn(:,2)-sgn(:,3))];
bus = p.nominal_bus_voltage*ones(size(t));
km = min(max(p.nominal_bus_voltage./bus,0.7),1.3);
cmd = (v-dist)./(bus.*km);
data = [cmd,iabc,iab,id,bus,km];
input = timeseries(data,t,'Name','Observer input');
truth.time = t; truth.theta = mod(theta,2*pi); truth.electrical_hz = f;
truth.active_flux = p.Flux*ones(size(t));
end
