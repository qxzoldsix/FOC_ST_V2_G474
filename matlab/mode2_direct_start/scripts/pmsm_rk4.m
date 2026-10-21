function xn=pmsm_rk4(x,v,q)
% Four RK4 substeps per PWM period; electrical/mechanical model, not firmware.
xn=x; h=q.Ts/4;
for k=1:4
    a=rhs(xn,v,q); b=rhs(xn+h*a/2,v,q);
    c=rhs(xn+h*b/2,v,q); d=rhs(xn+h*c,v,q);
    xn=xn+h*(a+2*b+2*c+d)/6;
end
xn(4)=mod(xn(4),2*pi);
end
function dx=rhs(x,v,q)
c=cos(x(4)); s=sin(x(4)); we=q.pole_pairs*x(3);
vd=v(1)*c+v(2)*s; vq=-v(1)*s+v(2)*c;
te=1.5*q.pole_pairs*(q.Flux*x(2)+(q.Ld-q.Lq)*x(1)*x(2));
dx=[(vd-q.Rs*x(1)+we*q.Lq*x(2))/q.Ld; ...
    (vq-q.Rs*x(2)-we*(q.Ld*x(1)+q.Flux))/q.Lq; ...
    (te-q.B*x(3)-q.load_torque)/q.J; we];
end
