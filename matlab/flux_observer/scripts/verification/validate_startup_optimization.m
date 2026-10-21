function validate_startup_optimization()
% Compare original zero reset with aligned flux seeding, same inputs and PI.
d = flux_observer_paths();
previous_filegen = Simulink.fileGenControl('getConfig');
restore_filegen = onCleanup(@()Simulink.fileGenControl('setConfig','config',previous_filegen));
Simulink.fileGenControl('set','CacheFolder',d.work,'CodeGenFolder',d.work,'createDir',true);
optimized = run_flux_observer_demo(false,'aligned');
model = 'G474_Hybrid_Flux_Observer'; ws = get_param(model,'ModelWorkspace');
restore_model = onCleanup(@()restore_aligned(model,optimized,d));
p = optimized.parameters; input = optimized.input; t = optimized.truth.time;
assert(isequal(p,flux_observer_params()),'Physical parameters or PI gains changed.');
zero_state = flux_observer_init(p,'source');
assignin(ws,'s0',zero_state); assignin(ws,'startup_mode','source');
source_sim = sim(model); original = source_sim.observer_y.Data;
reference = zeros(size(original)); state = zero_state;
for k=1:size(original,1)
    [v,state] = flux_observer_step(input.Data(k,:),state,p); reference(k,:) = v.';
end
source_difference = max(abs(original-reference),[],'all');
assert(source_difference<1e-8,'Original source reset no longer reproduces its MATLAB equations.');
startup = t<=0.4; target = optimized.truth.electrical_hz;
original_error = abs(original(:,2)-target);
optimized_error = abs(optimized.output(:,2)-target);
bad = find(startup & original_error>0.02*abs(target),1,'last');
old_settle = 0; if ~isempty(bad), old_settle=t(bad); end
bad = find(startup & optimized_error>0.02*abs(target),1,'last');
new_settle = 0; if ~isempty(bad), new_settle=t(bad); end
old_overshoot = max(0,max((original(startup,2)-target(startup))./target(startup)))*100;
new_overshoot = max(0,max((optimized.output(startup,2)-target(startup))./target(startup)))*100;
assert(new_settle<0.05 && new_settle<old_settle/2,'Startup settling did not improve.');
assert(new_overshoot<1,'Aligned startup overshoot exceeds 1 percent.');
assert(optimized.initial_state.pll_integral==0 && optimized.initial_state.speed_hz==0, ...
    'True or commanded speed must not be preloaded.');
scopes = find_system([model '/03_Results'],'SearchDepth',1,'BlockType','Scope');
assert(numel(scopes)==11,'Keep eleven independent scopes.');
fig = figure('Visible','off','Color','w','Position',[100 100 1150 760]);
tiledlayout(2,1,'TileSpacing','compact');
nexttile;
plot(t(startup),original(startup,3),'LineWidth',1); hold on;
plot(t(startup),optimized.output(startup,3),'LineWidth',1.5);
plot(t(startup),target(startup)*60/(p.MOTOR_POLES/2),'k--');
grid on; ylabel('Mechanical speed (rpm)');
legend('Original zero-flux reset','Aligned flux initialization','Reference','Location','southeast');
title('Same input and PI gains; initial flux state is the only change');
nexttile;
plot(t(startup),hypot(original(startup,4),original(startup,5))); hold on;
plot(t(startup),hypot(optimized.output(startup,4),optimized.output(startup,5)),'LineWidth',1.5);
yline(p.Flux,'k--'); grid on; xlabel('Time (s)'); ylabel('Active flux magnitude (Wb)');
legend('Original reset','Aligned initialization','PM flux parameter','Location','southeast');
exportgraphics(fig,fullfile(d.results,'startup_comparison.png'),'Resolution',150); close(fig);
fid = fopen(fullfile(d.results,'startup_comparison.txt'),'w'); cleanup = onCleanup(@()fclose(fid));
fprintf(fid,'Only initial stator flux changed; motor parameters and all PI gains unchanged.\n');
fprintf(fid,'Assumption: rotor has completed alignment at command angle 0 rad. No true angle/speed feedback.\n');
fprintf(fid,'PLL integral and speed estimate both start at zero.\n');
fprintf(fid,'Original 2-percent settling: %.9g s; overshoot: %.9g percent\n',old_settle,old_overshoot);
fprintf(fid,'Aligned 2-percent settling: %.9g s; overshoot: %.9g percent\n',new_settle,new_overshoot);
fprintf(fid,'Original reset native/MATLAB max difference: %.12g\n',source_difference);
fprintf(fid,'Aligned reset native/MATLAB max difference: %.12g\n',optimized.metrics.native_vs_matlab_max);
fprintf(fid,'Eleven independent scopes retained.\n');
fprintf('STARTUP_OPTIMIZATION_OK: %.6g s -> %.6g s; overshoot %.6g%% -> %.6g%%\n', ...
    old_settle,new_settle,old_overshoot,new_overshoot);
end

function restore_aligned(model,result,d)
if bdIsLoaded(model)
    ws = get_param(model,'ModelWorkspace');
    assignin(ws,'s0',result.initial_state); assignin(ws,'startup_mode','aligned');
    save_system(model,fullfile(d.model,[model '.slx']));
end
end
