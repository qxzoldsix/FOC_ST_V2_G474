function result=run_mode2(q_override,save_result)
if nargin<2, save_result=true; end
d=mode2_paths(); model='G474_Mode2_Direct_Start_Repair';
if ~bdIsLoaded(model), load_system(fullfile(d.model,[model '.slx'])); end
ws=get_param(model,'ModelWorkspace'); [p,q,s0]=mode2_params();
if nargin>=1 && ~isempty(q_override)
    q=q_override;
    % Explicit simulation conditions must remain reproducible (angle sweeps etc.).
    q.random_initial_angle=false;
end
q.initial_state=[0;0;q.initial_rpm*2*pi/60;q.initial_theta_e];
assignin(ws,'p',p); assignin(ws,'q',q); assignin(ws,'s0',s0);
old=Simulink.fileGenControl('getConfig'); restore=onCleanup(@()Simulink.fileGenControl('setConfig','config',old));
Simulink.fileGenControl('set','CacheFolder',d.work,'CodeGenFolder',d.work,'createDir',true);
output=sim(model); data=output.mode2_y;
q=getVariable(ws,'q'); % InitFcn chooses the actual physical initial angle.
values=squeeze(data.Data);
if size(values,1)~=numel(data.Time), values=values.'; end
assert(size(values,2)==34 && all(isfinite(values),'all'),'Invalid simulation output: %s',mat2str(size(values)));
result.time=data.Time; result.y=values; result.p=p; result.q=q;
result.initial_observer=s0; result.fSin=getVariable(ws,'fSin');
result.metrics=mode2_metrics(result);
if save_result
    save(fullfile(d.results,'mode2_results.mat'),'result');
    mode2_plot(result);
    torque_diagnosis_plot(result);
    compare_startup(result);
    save_system(model);
    fid=fopen(fullfile(d.results,'simulation_report.txt'),'w'); clean=onCleanup(@()fclose(fid));
    fprintf(fid,'MODE=2, source c43e794, source PI gains and equations unchanged.\n');
    fprintf(fid,'No VF, preposition, flux seed or true-angle/speed feedback.\n');
    fprintf(fid,'Vbus=%.6g V, target=%.6g/%.6g/%.6g rpm; switches %.6g/%.6g s\n',q.Vbus,q.target_rpm,q.second_rpm,q.third_rpm,q.switch_1,q.switch_2);
    fprintf(fid,'ASSUMED plant: J=%.9g, B=%.9g, load=%.9g Nm, theta0=%.9g deg\n',q.J,q.B,q.load_torque,q.initial_theta_e*180/pi);
    fprintf(fid,'Random physical initial angle enabled=%d; recorded theta0 is the actual simulation angle.\n',q.random_initial_angle);
    fprintf(fid,'Repaired Iq sign=%g (source -1); sampled winding-current sign=%g\n',q.speed_iq_sign,q.current_channel_sign);
    fprintf(fid,'Bridge is PWM-period average; original CCR conversion retained; assumed deadtime %.6g V.\n',q.actual_deadtime_V);
    fprintf(fid,'Observer reconstructs applied voltage from known previous command/CCR and previous measured current.\n');
    fprintf(fid,'Deadtime correction is applied ONCE upstream; fresh Clarke samples enter the observer.\n');
    fprintf(fid,'Two-pole reference filter %.6g Hz before source 1000 rpm/s slew limiter; PI Kp/Ki unchanged.\n',q.reference_filter_hz);
    fprintf(fid,'Observer warmup: zero reference for %.6g s; then normal reference released, no alignment current.\n',q.reference_start_delay_s);
    fprintf(fid,'Passive load: Tload=%.6g*tanh(wm/%.6g) Nm; scale=0 selects constant external load.\n',q.load_torque,q.load_speed_scale_rad_s);
    fprintf(fid,'Deadtime voltage FEEDFORWARD: %.6g V, enable %.6g s + ramp %.6g s, estimated-speed gate 80..160 rpm.\n',q.deadtime_feedforward_V,q.feedforward_start_s,q.feedforward_ramp_s);
    fprintf(fid,'All eleven scopes present. Controller sees no physical angle/rpm.\n\n');
    names=fieldnames(result.metrics);
    for k=1:numel(names), fprintf(fid,'%s = %.12g\n',names{k},result.metrics.(names{k})); end
    fprintf('MODE2_SIM_OK: actual final %.4f rpm; estimated %.4f rpm; peak |Iphase| %.4f A\n', ...
        result.metrics.final_actual_rpm,result.metrics.final_estimated_rpm,result.metrics.peak_phase_A);
end
end

