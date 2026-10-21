function result=verify_mode2()
d=mode2_paths(); [~,nominal]=mode2_params(); result=run_mode2(nominal);
model='G474_Mode2_Direct_Start_Repair'; ws=get_param(model,'ModelWorkspace');
restore=onCleanup(@()restore_default(model,ws,result));
q=result.q; q.stop_time=0.03; r=run_mode2(q,false);
ref=mode2_reference(r.p,r.q,r.fSin);
difference=max(abs(r.y-ref),[],'all');
assert(difference<1e-7,'Native/reference integration mismatch: %.12g',difference);
assert(all(result.y(:,32)==2),'Mode must remain 2.');
assert(all(result.y(1,[1:5 8:9 12 15:19])==0),'Zero-start conditions violated.');
assert(all(result.initial_observer.psi_s==0),'No flux seed allowed.');
assert(numel(find_system(model,'LookUnderMasks','all','BlockType','Scope'))==11,'Eleven scopes required.');
assert(isempty(find_system(model,'LookUnderMasks','all','BlockType','FromWorkspace')),'No prescribed physical speed/current playback.');
warmup=result.time<result.q.reference_start_delay_s;
assert(all(abs(result.y(warmup,12))<1e-8),'No rotor motion is allowed during zero-reference warmup.');
assert(all(abs(result.y(warmup,15:17))<1e-8,'all'),'Warmup must not apply alignment current.');
assert(all(abs(result.y(warmup,22:25))<1e-8,'all'),'Warmup must keep Iq/reference/PWM voltage command zero.');
% An active-start prefix check complements the zero-reference initial check.
q=result.q; q.stop_time=0.35; active=run_mode2(q,false);
ref=mode2_reference(active.p,active.q,active.fSin);
speed_difference=max(abs(active.y(:,[3 12])-ref(:,[3 12])),[],'all');
assert(speed_difference<0.2,'Active-start native/reference speed mismatch %.9g rpm',speed_difference);
actual=[result.metrics.stage1_actual_rpm result.metrics.stage2_actual_rpm result.metrics.stage3_actual_rpm];
targets=[result.q.target_rpm result.q.second_rpm result.q.third_rpm];
assert(all(abs(actual-targets)<2),'Nominal stage tracking must be within 2 rpm: %s',mat2str(actual));
assert(result.metrics.startup_min_rpm>-1,'Nominal start must not backspin by more than 1 rpm.');
assert(result.metrics.startup_overshoot_percent<5,'Nominal startup overshoot must be below 5 percent.');
assert(result.metrics.tail_torque_pp_Nm<0.0015,'Nominal torque ripple exceeds 1.5 mNm p-p.');
fid=fopen(fullfile(d.results,'verification_report.txt'),'w'); closeFile=onCleanup(@()fclose(fid));
fprintf(fid,'PASS native blocks / independent reference first30ms max %.12g\n',difference);
fprintf(fid,'PASS zero speed/current/observer states; MODE=2; no preposition/VF/true-angle feedback; 11 scopes.\n');
fprintf(fid,'PASS original PI equations and gains. Command polarity/shaping and voltage interfaces repaired.\n');
fprintf(fid,'PASS 150-ms zero-reference warmup: zero physical current/torque command/speed; active-start native/reference max %.12g rpm\n',speed_difference);
fprintf(fid,'PASS nominal stage actual %.9g / %.9g / %.9g rpm\n',actual);
fprintf(fid,'PASS nominal startup min %.9g rpm, overshoot %.9g percent, settle2pct %.9g s\n', ...
    result.metrics.startup_min_rpm,result.metrics.startup_overshoot_percent,result.metrics.startup_settle_2percent_s);
fprintf(fid,'PASS unfiltered torque mean %.9g Nm, p-p %.9g Nm\n',result.metrics.tail_torque_mean_Nm,result.metrics.tail_torque_pp_Nm);
% Initial-position audit. Failures remain explicit; no true-angle initialization.
angles=[0 30 90 180 270]; rows=zeros(numel(angles),7);
for k=1:numel(angles)
    q=result.q; q.initial_theta_e=angles(k)*pi/180; q.initial_state(4)=q.initial_theta_e;
    y=mode2_reference(result.p,q,result.fSin); test=result; test.q=q; test.y=y; m=mode2_metrics(test);
    rows(k,:)=[angles(k),m.stage1_actual_rpm,m.stage2_actual_rpm,m.stage3_actual_rpm,m.startup_min_rpm,m.tail_torque_pp_Nm,m.tracks_final_target];
    fprintf(fid,'Initial-angle audit %g deg: %.6g / %.6g / %.6g rpm; min %.6g rpm; Tpp %.6g; final tracking %d\n',rows(k,:));
end
writetable(array2table(rows,'VariableNames',{'angle_deg','stage1_rpm','stage2_rpm','stage3_rpm','startup_min_rpm','torque_pp_Nm','final_tracks'}),fullfile(d.results,'initial_angle_sweep.csv'));
% Feedforward calibration sensitivity, using the same observer estimate.
for physical_deadtime=[0.24 0.36]
    q=result.q; q.actual_deadtime_V=physical_deadtime;
    y=mode2_reference(result.p,q,result.fSin); test=result; test.q=q; test.y=y; m=mode2_metrics(test);
    fprintf(fid,'Deadtime mismatch actual %.3g V, estimate .3 V: final %.6g rpm; Tpp %.6g Nm; tracks %d\n', ...
        physical_deadtime,m.final_actual_rpm,m.tail_torque_pp_Nm,m.tracks_final_target);
end
fprintf(fid,'LIMITATION: initial position is unknown at zero speed. Passing nominal tests is not a guarantee for every initial angle.\n');
fprintf('REPAIR_VERIFICATION_OK: nominal stages %.4f / %.4f / %.4f rpm; raw torque p-p %.6g Nm\n',actual,result.metrics.tail_torque_pp_Nm);
end
function restore_default(model,ws,r)
q=r.q; q.random_initial_angle=true;
assignin(ws,'p',r.p); assignin(ws,'q',q); assignin(ws,'s0',r.initial_observer); save_system(model);
end
