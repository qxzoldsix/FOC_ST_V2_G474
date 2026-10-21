function validate_flux_observer()
% Compare expanded native blocks, MATLAB equations, and original float C.
d = flux_observer_paths();
previous_filegen = Simulink.fileGenControl('getConfig');
restore_filegen = onCleanup(@()Simulink.fileGenControl('setConfig','config',previous_filegen));
Simulink.fileGenControl('set','CacheFolder',d.work, ...
    'CodeGenFolder',d.work,'createDir',true);
result = run_flux_observer_demo(false,'source');
config = mex.getCompilerConfigurations('C','Selected');
c_available = ~isempty(config);
backend = 'mex';
if c_available
    mex('-silent','-outdir',d.work,fullfile(d.verification,'observer_firmware_mex.c'));
    addpath(d.work);
    compiler_name = config.Name;
else
    % MATLAB bundles LCC64 for its own C builds. Use a standalone console
    % adapter without changing the user's persistent MEX configuration.
    lccroot = fullfile(matlabroot,'sys','lcc64','lcc64');
    cc = fullfile(lccroot,'bin','lcc64.exe');
    linker = fullfile(lccroot,'bin','lcclnk64.exe');
    if isfile(cc) && isfile(linker)
        obj = fullfile(d.work,'observer_firmware_host.obj');
        exe = fullfile(d.work,'observer_firmware_host.exe');
        cmd = sprintf('"%s" -c -noregistrylookup -I"%s" -Fo"%s" "%s"', ...
            cc,fullfile(lccroot,'include64'),obj,fullfile(d.verification,'observer_firmware_host.c'));
        [status,message] = system(cmd); assert(status==0,'%s',message);
        cmd = sprintf('"%s" -L"%s" -o "%s" "%s"',linker,fullfile(lccroot,'lib64'),exe,obj);
        [status,message] = system(cmd); assert(status==0,'%s',message);
        c_available = true; backend = 'host'; compiler_name = 'MATLAB bundled LCC64 (standalone)';
    end
end
model = 'G474_Hybrid_Flux_Observer'; p = result.parameters;
restore_model = onCleanup(@()restore_default_model(model,result,d.model));
cases = {'PI demo','PI reverse and bus scaling','PI deadband','SMO demo'};
fid = fopen(fullfile(d.results,'validation_report.txt'),'a'); cleanup = onCleanup(@()fclose(fid));
if c_available
    fprintf(fid,'C compiler: %s\n',compiler_name);
else
    fprintf(fid,'Original C regression SKIPPED: no configured C MEX compiler.\n');
    fprintf('Original C regression skipped: no configured C MEX compiler.\n');
end
for c=1:numel(cases)
    data = result.input.Data;
    if c==2
        data(:,2) = -data(:,2); data(:,4:5) = data(:,[5 4]);
        data(:,7) = -data(:,7);
        oldscale = data(:,9).*data(:,10);
        data(:,9) = 8+16*(0.5+0.5*sin(result.truth.time*7));
        data(:,10) = min(max(p.nominal_bus_voltage./data(:,9),0.7),1.3);
        data(:,1:2) = data(:,1:2).*oldscale./(data(:,9).*data(:,10));
    elseif c==3
        data(:,1:8) = 0;
        edges = [-0.011,-0.01,-0.009,0,0.009,0.01,0.011];
        for k=1:3, data(:,k+2) = edges(mod((1:size(data,1))+k,numel(edges))+1); end
    end
    p.Comp_Mode = double(c==4);
    % With C enabled, compare using float-representable input.
    if c_available, data = double(single(data)); end
    input = timeseries(data,result.truth.time);
    ws = get_param(model,'ModelWorkspace'); assignin(ws,'p',p); assignin(ws,'observer_input',input);
    out = sim(model); native = out.observer_y.Data;
    s = flux_observer_init(p); reference = zeros(size(native));
    for k=1:size(data,1)
        [v,s] = flux_observer_step(data(k,:),s,p); reference(k,:) = v.';
    end
    native_difference = native-reference;
    native_difference(:,1) = atan2(sin(native_difference(:,1)),cos(native_difference(:,1)));
    exact = max(abs(native_difference),[],'all');
    % Absolute tolerances include the RPM channel (up to thousands of RPM).
    assert(exact<1e-6,'Native block regression failed: %s (max %.12g)',cases{c},exact);
    assert(all(isfinite(native(:))),'Non-finite output: %s',cases{c});
    assert(all(native(:,1)>=0 & native(:,1)<2*pi),'Angle wrap failed: %s',cases{c});
    assert(max(abs(native(:,9)))<=p.pll_step_limit+1e-12,'PLL limit failed: %s',cases{c});
    % Speed is held on all samples except every eighth observer invocation.
    held = mod((2:size(native,1)).',p.speed_decimation)~=0;
    delta_hz = diff(native(:,2));
    assert(all(delta_hz(held)==0),'Eight-cycle speed update failed: %s',cases{c});
    if c==3
        sg = double(data(:,3:5)>p.current_deadband)-double(data(:,3:5)<-p.current_deadband);
        expected_v = [0.6666667*p.V_deadtime*(sg(:,1)-0.5*sg(:,2)-0.5*sg(:,3)), ...
            0.5773503*p.V_deadtime*(sg(:,2)-sg(:,3))];
        assert(max(abs(native(:,10:11)-expected_v),[],'all')<1e-12,'Deadtime boundary failed.');
    end
    if c_available
    if strcmp(backend,'mex')
        firmware = observer_firmware_mex(data,p.Comp_Mode);
    else
        infile = [tempname '.bin']; outfile = [tempname '.bin'];
        f = fopen(infile,'wb'); fwrite(f,data.','double'); fclose(f);
        cmd = sprintf('"%s" "%s" "%s" %d',exe,infile,outfile,p.Comp_Mode);
        [status,message] = system(cmd); assert(status==0,'%s',message);
        f = fopen(outfile,'rb'); firmware = fread(f,[11 Inf],'double').'; fclose(f);
        delete(infile); delete(outfile);
        assert(isequal(size(firmware),size(reference)),'C output size mismatch.');
    end
    fp = flux_observer_params('single'); fp.Comp_Mode = single(p.Comp_Mode);
    fs = flux_observer_init(fp); float_reference = zeros(size(reference));
    for k=1:size(data,1)
        [v,fs] = flux_observer_step(single(data(k,:)),fs,fp);
        float_reference(k,:) = double(v.');
    end
    dtheta = atan2(sin(float_reference(:,1)-firmware(:,1)),cos(float_reference(:,1)-firmware(:,1)));
    speederr = max(abs(float_reference(:,2)-firmware(:,2)));
    fluxerr = max(abs(float_reference(:,4:7)-firmware(:,4:7)),[],'all');
    fprintf('C compare %s: angle %.9g rad; speed %.9g Hz; flux %.9g Wb\n', ...
        cases{c},max(abs(dtheta)),speederr,fluxerr);
    double_angle = atan2(sin(reference(:,1)-firmware(:,1)),cos(reference(:,1)-firmware(:,1)));
    fprintf(fid,'%s double/float startup max angle %.9g rad; after 0.3s %.9g rad\n', ...
        cases{c},max(abs(double_angle)),max(abs(double_angle(result.truth.time>0.3))));
    settled = result.truth.time>0.3;
    settled_angle = max(abs(dtheta(settled)));
    settled_speed = max(abs(float_reference(settled,2)-firmware(settled,2)));
    fprintf('C single after 0.3s: %.9g rad, %.9g Hz\n',settled_angle,settled_speed);
    % Float trig libraries and intermediate rounding affect the high-gain
    % zero-flux startup. Report its full error; compare settled trajectories
    % separately instead of claiming bit identity across host runtimes.
    angle_tolerance = 0.001;
    if p.Comp_Mode~=0, angle_tolerance = 0.002; end % nonsmooth SMO feedback
    assert(settled_angle<angle_tolerance && settled_speed<0.02 && fluxerr<5e-5 && ...
        max(abs(dtheta))<0.15 && speederr<3, ...
        'Original C trajectory regression failed: %s',cases{c});
    assert(max(abs(float_reference(:,10:11)-firmware(:,10:11)),[],'all')<1e-6, ...
        'Original C voltage reconstruction failed: %s',cases{c});
    fprintf(fid,'%s: blocks/MATLAB %.6g; C single full angle %.6g rad, speed %.6g Hz, flux %.6g Wb; after 0.3s angle %.6g rad, speed %.6g Hz\n', ...
        cases{c},exact,max(abs(dtheta)),speederr,fluxerr,settled_angle,settled_speed);
    else
        fprintf(fid,'%s: blocks/MATLAB max absolute difference %.9g; angle/limit/speed-hold checks passed\n',cases{c},exact);
    end
    fprintf('%s PASSED\n',cases{c});
end
fprintf(fid,'All four cases PASSED. Model restored to default PI configuration.\n');
validate_firmware_pi();
end

function restore_default_model(model,result,here)
if bdIsLoaded(model)
    ws = get_param(model,'ModelWorkspace');
    assignin(ws,'p',result.parameters); assignin(ws,'observer_input',result.input);
    assignin(ws,'s0',result.initial_state); assignin(ws,'startup_mode',result.startup_mode);
    save_system(model,fullfile(here,[model '.slx']));
end
end
