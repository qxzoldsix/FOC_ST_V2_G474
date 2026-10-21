function torque_diagnosis_plot(r)
d=mode2_paths(); t=r.time; y=r.y; ix=t>r.q.stop_time-0.2;
iq_true=-y(:,33).*sin(y(:,13))+y(:,34).*cos(y(:,13));
f=figure('Visible','off','Color','w','Position',[100 100 1350 900]); clean=onCleanup(@()close(f));
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
nexttile; plot(t(ix),y(ix,14)*1000); grid on; ylabel('Torque (mN m)');
title(sprintf('RAW steady torque: mean %.4g, peak-to-peak %.4g mN m', ...
    r.metrics.tail_torque_mean_Nm*1000,r.metrics.tail_torque_pp_Nm*1000));
nexttile; plot(t(ix),iq_true(ix),t(ix),y(ix,19)); grid on; ylabel('Iq (A)');
legend('Physical Iq, analysis only','Controller Iq feedback','Location','best'); title('Torque follows physical Iq');
nexttile; plot(t(ix),atan2(sin(y(ix,1)-y(ix,13)),cos(y(ix,1)-y(ix,13)))*180/pi);
grid on; ylabel('Electrical angle error (deg)'); title('Observer minus physical angle');
nexttile; plot(t(ix),y(ix,26),t(ix),y(ix,10)); grid on; ylabel('Alpha voltage (V)');
legend('Applied bridge','Observer reconstruction (previous cycle)','Location','best'); title('Voltage consistency');
for k=1:4, ax=nexttile(k); xlabel(ax,'Time (s)'); end
sgtitle('Torque diagnosis: unfiltered physical torque; no display smoothing');
exportgraphics(f,fullfile(d.results,'torque_diagnosis.png'),'Resolution',140);
% Preserve a clear numerical before/after reference, including the failed
% temporary 2-A-cap experiment that reproduces the user screenshot.
file=fullfile(d.work,'limit_2.mat');
if isfile(file)
    old=load(file); a=old.rr; oldtail=a.time>a.q.stop_time-0.2;
    f2=figure('Visible','off','Color','w','Position',[100 100 1350 650]); clean2=onCleanup(@()close(f2));
    tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
    nexttile; plot(a.time(oldtail),a.y(oldtail,14)*1000); grid on; xlabel('Time (s)'); ylabel('mN m');
    title('Failed startup / temporary 2-A command limit');
    nexttile; plot(t(ix),y(ix,14)*1000); grid on; xlabel('Time (s)'); ylabel('mN m');
    title('Repaired nominal startup / source 12-A limit');
    exportgraphics(f2,fullfile(d.results,'torque_before_after.png'),'Resolution',140);
end
end
