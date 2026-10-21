function result = run_flux_observer_demo(show_figures,startup_mode)
% Run directly in MATLAB: run_flux_observer_demo
if nargin==0, show_figures = true; end
if nargin<2, startup_mode='aligned'; end
d = flux_observer_paths();
% Keep simulation caches beside this artifact, even when called elsewhere.
previous_filegen = Simulink.fileGenControl('getConfig');
restore_filegen = onCleanup(@()Simulink.fileGenControl('setConfig','config',previous_filegen));
Simulink.fileGenControl('set','CacheFolder',d.work, ...
    'CodeGenFolder',d.work,'createDir',true);
model = build_flux_observer(d.model,startup_mode);
p = flux_observer_params(); [input,truth] = flux_observer_demo_input(p);
initial_state = flux_observer_init(p,startup_mode,input.Data(1,:),0);
simulation = sim(model);
y = simulation.observer_y.Data;
reference = zeros(size(y)); s = initial_state;
for k=1:size(y,1)
    [v,s] = flux_observer_step(input.Data(k,:),s,p);
    reference(k,:) = v.';
end
parity = max(abs(y-reference),[],1);
assert(max(parity)<1e-8,'Simulink and MATLAB discrete steps disagree.');
assert(all(isfinite(y(:))),'Non-finite observer output.');
assert(all(y(:,1)>=0 & y(:,1)<2*pi),'Angle wrap failed.');
assert(max(abs(y(:,9)))<=p.pll_step_limit+1e-12,'PLL step limit failed.');
angle_error = atan2(sin(truth.theta-y(:,1)),cos(truth.theta-y(:,1)));
amplitude = hypot(y(:,4),y(:,5));
tail = truth.time>1.85;
metrics.native_vs_matlab_max = max(parity);
metrics.final_speed_hz = y(end,2);
metrics.final_true_speed_hz = truth.electrical_hz(end);
metrics.tail_angle_rms_deg = sqrt(mean(angle_error(tail).^2))*180/pi;
metrics.tail_flux_mean_wb = mean(amplitude(tail));
metrics.tail_flux_relative_error = abs(mean(amplitude(tail))-p.Flux)/p.Flux;
assert(abs(metrics.final_speed_hz-metrics.final_true_speed_hz)<1,'Speed did not converge.');
assert(metrics.tail_angle_rms_deg<10,'Angle tracking did not converge.');
assert(metrics.tail_flux_relative_error<0.1,'Flux amplitude did not converge.');
visibility = 'off'; if show_figures, visibility = 'on'; end
fig = figure('Name','G474 Hybrid Flux Observer','Color','w','Visible',visibility, ...
    'Position',[100 100 1200 850]);
tiledlayout(3,2,'TileSpacing','compact');
nexttile; plot(truth.time,[truth.electrical_hz,y(:,2)]); grid on;
ylabel('Electrical Hz'); legend('True','Estimated');
nexttile; plot(truth.time,angle_error*180/pi); grid on; ylabel('Angle error (deg)');
nexttile; plot(truth.time,[amplitude,truth.active_flux]); grid on;
ylabel('Active flux (Wb)'); legend('Estimated','PM reference');
nexttile; plot(y(:,4),y(:,5)); axis equal; grid on;
xlabel('Active flux alpha (Wb)'); ylabel('Active flux beta (Wb)');
nexttile; plot(truth.time,[y(:,8),y(:,9)]); grid on;
legend('PLL error (Wb)','PLL step (rad/sample)'); xlabel('Time (s)');
nexttile; plot(truth.time,y(:,10:11)); grid on;
ylabel('Corrected voltage (V)'); xlabel('Time (s)'); legend('alpha','beta');
sgtitle(sprintf('c43e794 | Hybrid Active Flux | 20 kHz | startup: %s | unchanged PI',startup_mode));
exportgraphics(fig,fullfile(d.results,'flux_observer_demo.png'),'Resolution',150);
if ~show_figures, close(fig); end
result.parameters = p; result.input = input; result.truth = truth;
result.output = y; result.metrics = metrics;
result.startup_mode = startup_mode; result.initial_state = initial_state;
save(fullfile(d.results,'flux_observer_demo_results.mat'),'result');
report = 'simulation_report.txt';
if strcmp(startup_mode,'source'), report='validation_report.txt'; end
fid = fopen(fullfile(d.results,report),'w');
cleanup = onCleanup(@()fclose(fid));
fprintf(fid,'Source: %s\n',p.source_commit);
fprintf(fid,'Startup: %s (aligned mode requires completed rotor alignment)\n',startup_mode);
fprintf(fid,'MATLAB: %s\n',version);
fprintf(fid,'Simulink/MATLAB max absolute difference: %.12g\n',metrics.native_vs_matlab_max);
fprintf(fid,'Final estimated/true electrical Hz: %.9g / %.9g\n',metrics.final_speed_hz,metrics.final_true_speed_hz);
fprintf(fid,'Tail angle RMS (deg): %.9g\n',metrics.tail_angle_rms_deg);
fprintf(fid,'Tail active flux mean (Wb): %.9g\n',metrics.tail_flux_mean_wb);
fprintf(fid,'Tail active flux relative error: %.9g\n',metrics.tail_flux_relative_error);
disp(metrics);
if show_figures, open_system(model); end
end
