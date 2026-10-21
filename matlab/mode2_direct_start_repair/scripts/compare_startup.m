function summary=compare_startup(result)
% Before/after comparison of raw signals, retaining initialization time.
d=mode2_paths(); file=fullfile(d.work,'startup_before.mat');
if ~isfile(file), summary=[]; return; end
saved=load(file); before=saved.baseline; after=result;
if abs(mod(after.q.initial_theta_e-before.q.initial_theta_e+pi,2*pi)-pi)>1e-10
    summary=[]; % Different initial positions cannot be compared as a tuning improvement.
    fprintf('Startup before/after comparison skipped: initial electrical angles differ.\n');
    return;
end
summary.before=measure(before); summary.after=measure(after);
fid=fopen(fullfile(d.results,'startup_optimization_report.txt'),'w'); closeFile=onCleanup(@()fclose(fid));
fprintf(fid,'MODE=2 startup optimization: zero reference for 150 ms, then release the SAME reference filters/ramp.\n');
fprintf(fid,'PI gains/formulas, observer, speed filter, plant, load and target switch times unchanged.\n');
fprintf(fid,'No alignment current, VF, true-angle initialization, speed preloading or display smoothing.\n');
names=fieldnames(summary.before);
for k=1:numel(names)
    fprintf(fid,'%s: before %.12g; after %.12g\n',names{k},summary.before.(names{k}),summary.after.(names{k}));
end
fprintf(fid,'Tradeoff: physical acceleration begins 150 ms later; no startup data is removed.\n');
save(fullfile(d.results,'startup_optimization_results.mat'),'summary');
f=figure('Visible','off','Color','w','Position',[100 100 1450 1000]); closeFig=onCleanup(@()close(f));
tiledlayout(2,2,'Padding','compact','TileSpacing','compact');
nexttile; plot(before.time,before.y(:,12),after.time,after.y(:,12),'LineWidth',1.15); hold on;
yline(after.q.target_rpm,'k--'); xlim([0 0.8]); grid on; ylabel('Actual speed (rpm)'); xlabel('Time from MODE=2 entry (s)');
legend('Before','After','200 rpm command','Location','best'); title('Physical motor speed: complete startup interval');
nexttile; plot(before.time,before.y(:,3),after.time,after.y(:,3),'LineWidth',1.15); hold on;
yline(after.q.target_rpm,'k--'); xlim([0 0.8]); grid on; ylabel('Estimated speed (rpm)'); xlabel('Time from MODE=2 entry (s)');
legend('Before','After','200 rpm command','Location','best'); title('Raw observer output: no additional speed filtering');
nexttile;
plot(before.time-summary.before.reference_delay_s,before.y(:,3)-before.y(:,12), ...
    after.time-summary.after.reference_delay_s,after.y(:,3)-after.y(:,12),'LineWidth',1.05);
xlim([0 0.45]); grid on; ylabel('Estimated minus actual (rpm)'); xlabel('Time after reference release (s)');
legend('Before','After','Location','best'); title('Aligned active-start window: idle time excluded from error comparison');
nexttile; plot(after.time,after.y(:,31),'k--',after.time,after.y(:,23),after.time,after.y(:,12),after.time,after.y(:,3));
grid on; xlabel('Time (s)'); ylabel('rpm'); legend('Command','Released/shaped reference','Actual','Estimated','Location','best');
title('Full 200 / 400 / 1000 rpm test');
sgtitle('Startup optimization: observer initialization at zero reference, then direct MODE=2 closed loop');
exportgraphics(f,fullfile(d.results,'startup_optimization.png'),'Resolution',135);
end
function m=measure(r)
q=r.q; t=r.time; y=r.y; ix=t<q.switch_1;
delay=0; if isfield(q,'reference_start_delay_s'), delay=q.reference_start_delay_s; end
active=t>=delay & t<delay+0.45;
m.reference_delay_s=delay;
m.estimated_startup_peak_rpm=max(y(ix,3));
m.actual_startup_peak_rpm=max(y(ix,12));
m.estimated_overshoot_pct=max(0,(m.estimated_startup_peak_rpm/q.target_rpm-1)*100);
m.actual_overshoot_pct=max(0,(m.actual_startup_peak_rpm/q.target_rpm-1)*100);
m.active_start_max_estimation_error_rpm=max(abs(y(active,3)-y(active,12)));
m.active_start_estimation_RMSE_rpm=sqrt(mean((y(active,3)-y(active,12)).^2));
metrics=mode2_metrics(r);
m.settle2pct_from_mode2_entry_s=metrics.startup_settle_2percent_s;
m.settle2pct_after_reference_release_s=metrics.startup_settle_2percent_s-delay;
m.phase_current_peak_A=metrics.peak_phase_A;
m.tail_torque_pp_Nm=metrics.tail_torque_pp_Nm;
end
