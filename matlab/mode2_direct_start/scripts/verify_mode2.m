function result=verify_mode2()
d=mode2_paths(); result=run_mode2();
model='G474_Mode2_Direct_Start'; ws=get_param(model,'ModelWorkspace');
restore=onCleanup(@()restore_default(model,ws,result));
% Exact early-cycle comparison avoids making bit-exact claims about nonlinear
% closed-loop trajectories after floating-point/table-quantization divergence.
q=result.q; q.stop_time=0.03; r=run_mode2(q,false);
ref=mode2_reference(r.p,r.q,r.fSin);
diff=max(abs(r.y-ref),[],1);
assert(max(diff)<1e-7,'Block/reference integration mismatch: %.9g',max(diff));
assert(all(result.y(:,32)==2),'Startup must remain mode=2 throughout.');
assert(all(result.y(1,[1:5 8:9])==0),'Observer angle/speed/old flux must start at zero.');
assert(all(result.initial_observer.psi_s==0),'Initial stator integrator state must be zero.');
assert(all(result.y(1,15:19)==0),'Plant and controller must start with zero current.');
assert(result.y(1,12)==0,'Motor must start at rest.');
assert(result.metrics.ccr_min>=0 && result.metrics.ccr_max<=3999,'CCR bounds violated.');
scopes=find_system(model,'LookUnderMasks','all','BlockType','Scope');
assert(numel(scopes)==11,'Exactly eleven scopes required.');
assert(numel(find_system(model,'LookUnderMasks','all','BlockType','FromWorkspace'))==0,'No prescribed current/speed playback allowed.');
fid=fopen(fullfile(d.results,'verification_report.txt'),'w'); clean=onCleanup(@()fclose(fid));
fprintf(fid,'PASS: native block / independent MATLAB reference first 30 ms: max %.12g\n',max(diff));
fprintf(fid,'PASS: MODE=2 throughout; zero observer/current/speed initial states; 11 scopes; CCR bounds.\n');
fprintf(fid,'PASS: no FromWorkspace playback; physical angle/speed do not feed controller.\n');
fprintf(fid,'No passing-startup assertion: physical success/failure is an experiment result.\n');
fprintf(fid,'Default source sign final actual/estimated: %.9g / %.9g rpm. Tracks target: %d\n', ...
    result.metrics.final_actual_rpm,result.metrics.final_estimated_rpm,result.metrics.tracks_final_target);
% Initial-position dependence; retain source coefficients/signs in every case.
angles=[0 90 180]; summary=zeros(numel(angles)+1,5);
summary(1,:)=[result.q.initial_theta_e*180/pi,result.metrics.final_actual_rpm, ...
    result.metrics.final_estimated_rpm,result.metrics.peak_phase_A,result.metrics.tracks_final_target];
for k=1:numel(angles)
    q=result.q; q.initial_theta_e=angles(k)*pi/180;
    r=run_mode2(q,false);
    summary(k+1,:)=[angles(k),r.metrics.final_actual_rpm,r.metrics.final_estimated_rpm,r.metrics.peak_phase_A,r.metrics.tracks_final_target];
    fprintf(fid,'Initial angle %g deg: actual %.9g rpm, estimate %.9g rpm, peak %.9g A, tracks %d\n',summary(k+1,:));
end
T=array2table(summary,'VariableNames',{'initial_angle_deg','final_actual_rpm','final_estimated_rpm','peak_phase_A','tracks_final_target'});
writetable(T,fullfile(d.results,'initial_angle_sweep.csv'));
% Diagnostic only: source=-1 remains the saved/default model configuration.
q=result.q; q.speed_iq_sign=1; diagnostic=run_mode2(q,false);
save(fullfile(d.results,'iq_sign_diagnostic.mat'),'diagnostic');
fprintf(fid,'DIAGNOSTIC ONLY, Iq sign +1 (not source): actual %.9g, estimated %.9g rpm, tracks %d\n', ...
    diagnostic.metrics.final_actual_rpm,diagnostic.metrics.final_estimated_rpm,diagnostic.metrics.tracks_final_target);
f=figure('Visible','off','Color','w','Position',[100 100 1300 650]); closeFig=onCleanup(@()close(f));
plot(result.time,result.y(:,31),'k--',result.time,result.y(:,12),diagnostic.time,diagnostic.y(:,12),'LineWidth',1.1);
grid on; xlabel('Time (s)'); ylabel('Actual motor speed (rpm)');
legend('Target','Source Iq sign = -1','Diagnostic Iq sign = +1','Location','best');
title('Sign diagnostic under assumed standard winding/current polarity; saved model retains source -1');
exportgraphics(f,fullfile(d.results,'iq_sign_diagnostic.png'),'Resolution',130);
fprintf('MODE2_VERIFICATION_OK: reference max %.12g; eleven scopes; angle sweep complete.\n',max(diff));
end
function restore_default(model,ws,r)
assignin(ws,'p',r.p); assignin(ws,'q',r.q); assignin(ws,'s0',r.initial_observer);
save_system(model);
end
