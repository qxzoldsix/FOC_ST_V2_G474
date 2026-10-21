function mode2_plot(r)
d=mode2_paths(); t=r.time; y=r.y;
f=figure('Visible','off','Color','w','Position',[100 60 1500 1100]); c=onCleanup(@()close(f));
tiledlayout(3,2,'TileSpacing','compact','Padding','compact');
nexttile; plot(t,y(:,31),'k--',t,y(:,23),'Color',[0.5 0.5 0.5]); hold on;
plot(t,y(:,12),'LineWidth',1.15); plot(t,y(:,3),'LineWidth',0.9); grid on; ylabel('rpm');
legend('Command','Firmware ramp','Actual motor','Observer','Location','best'); title('MODE=2: direct closed-loop start');
nexttile; err=atan2(sin(y(:,1)-y(:,13)),cos(y(:,1)-y(:,13)))*180/pi;
plot(t,err); grid on; ylabel('electrical deg'); title('Estimated minus physical rotor angle');
nexttile; plot(t,y(:,22),t,y(:,19)); grid on; ylabel('A'); title('q axis current'); legend('Iq command','Iq feedback','Location','best');
nexttile; plot(t,y(:,15:17)); grid on; ylabel('A'); title('Measured phase currents'); legend('Iu','Iv','Iw','Location','best');
nexttile; plot(t,hypot(y(:,4),y(:,5))); hold on; yline(r.p.Flux,'k--'); grid on; ylabel('Wb'); title('Estimated active-flux magnitude');
nexttile; plot(t,y(:,26),t,y(:,10)); grid on; ylabel('V'); title('Actual bridge vs observer alpha voltage'); legend('Physical bridge','Observer input','Location','best');
for k=1:6, ax=nexttile(k); xlabel(ax,'Time (s)'); end
sgtitle(sprintf('12 V | 200 -> 400 -> 1000 rpm | load %.4g Nm | source SPEED IQ SIGN = %g | assumed J = %.3g', ...
    r.q.load_torque,r.q.speed_iq_sign,r.q.J));
exportgraphics(f,fullfile(d.results,'mode2_startup.png'),'Resolution',135);
end
