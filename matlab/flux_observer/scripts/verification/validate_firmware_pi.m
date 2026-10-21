function validate_firmware_pi()
% Check each actual PI_Cale.c instance, including equality and changing limits.
d = flux_observer_paths();
previous_filegen = Simulink.fileGenControl('getConfig');
restore_filegen = onCleanup(@()Simulink.fileGenControl('setConfig','config',previous_filegen));
Simulink.fileGenControl('set','CacheFolder',d.work,'CodeGenFolder',d.work,'createDir',true);
if ~bdIsLoaded('G474_Firmware_PI'), load_system(fullfile(d.model,'G474_Firmware_PI.slx')); end
lccroot = fullfile(matlabroot,'sys','lcc64','lcc64');
cc = fullfile(lccroot,'bin','lcc64.exe'); linker = fullfile(lccroot,'bin','lcclnk64.exe');
c_available = isfile(cc) && isfile(linker);
if c_available
    obj = fullfile(d.work,'pid_firmware_host.obj'); exe = fullfile(d.work,'pid_firmware_host.exe');
    cmd = sprintf('"%s" -c -noregistrylookup -I"%s" -Fo"%s" "%s"', ...
        cc,fullfile(lccroot,'include64'),obj,fullfile(d.verification,'pid_firmware_host.c'));
    [status,message] = system(cmd); assert(status==0,'%s',message);
    cmd = sprintf('"%s" -L"%s" -o "%s" "%s"',linker,fullfile(lccroot,'lib64'),exe,obj);
    [status,message] = system(cmd); assert(status==0,'%s',message);
end
fid = fopen(fullfile(d.results,'pi_validation_report.txt'),'w'); cleanup = onCleanup(@()fclose(fid));
fprintf(fid,'PI_Cale.c PID_controller, source c43e794\n');
names = {'FOC_Speed_PI','FOC_Id_PI','FOC_Iq_PI'};
kps = [0.01 0.01 0.01]; kis = [0.0001 0.00001 0.00001]; limits = [12 0.95 0.95];
for c=1:3
    n = 128; t = (0:n-1).'*5e-5;
    error = [123;0;-123;-123;1000*ones(20,1);-1000*ones(20,1); ...
        zeros(10,1);5*ones(20,1);-5*ones(20,1);200*ones(34,1)];
    data = [error,zeros(n,1),limits(c)*ones(n,1),-limits(c)*ones(n,1)];
    data(1:2,3) = 0; data(3:4,4) = 0; % exact equality to previous v1=0
    data(80:90,3) = 0.02; data(80:90,4) = -0.02;
    data(100:105,3:4) = 0; % q-axis zero voltage-circle margin
    model = 'G474_PI_Verification';
    if bdIsLoaded(model), close_system(model,0); end
    new_system(model); close_bench = onCleanup(@()close_system(model,0));
    set_param(model,'Solver','FixedStepDiscrete','FixedStep','5e-5', ...
        'StopTime',num2str(t(end),17),'ReturnWorkspaceOutputs','on');
    ws = get_param(model,'ModelWorkspace'); assignin(ws,'pi_input',timeseries(data,t));
    add_block('simulink/Sources/From Workspace',[model '/Input'],'VariableName','pi_input', ...
        'Interpolate','off','OutputAfterFinalValue','Holding final value');
    add_block('simulink/Signal Routing/Demux',[model '/Split'],'Outputs','4');
    add_block(['G474_Firmware_PI/' names{c}],[model '/Firmware_PI']);
    add_block('simulink/Signal Routing/Mux',[model '/Outputs'],'Inputs','4');
    add_block('simulink/Sinks/To Workspace',[model '/Log'],'VariableName','pi_y','SaveFormat','Timeseries');
    add_line(model,'Input/1','Split/1');
    for k=1:4
        add_line(model,sprintf('Split/%d',k),sprintf('Firmware_PI/%d',k));
        add_line(model,sprintf('Firmware_PI/%d',k),sprintf('Outputs/%d',k));
    end
    add_line(model,'Outputs/1','Log/1');
    output = sim(model); y = output.pi_y.Data;
    reference = zeros(n,4); integral = 0; previous_v1 = 0;
    frozen = false(n,1);
    for k=1:n
        e = data(k,1)-data(k,2); upper = data(k,3); lower = data(k,4);
        frozen(k) = previous_v1>=upper || previous_v1<=lower;
        if ~frozen(k), integral = integral+kis(c)*e; end
        integral = min(max(integral,lower),upper);
        previous_v1 = kps(c)*e+integral;
        reference(k,:) = [min(max(previous_v1,lower),upper),integral,previous_v1,e];
    end
    difference = max(abs(y-reference),[],'all');
    assert(difference<1e-10,'PI source equation mismatch: %s',names{c});
    assert(all(frozen(1:2)) && all(y(1:2,2)==0),'Upper equality must freeze integration.');
    assert(frozen(3) && y(3,2)==0,'Lower equality must freeze integration.');
    assert(all(y(100:105,1:2)==0,'all'),'Zero voltage margin must clamp Out and ui to zero.');
    fprintf(fid,'%s Kp=%.8g Ki=%.8g: native/MATLAB max %.9g; equality/freeze/dynamic-limit checks PASSED\n', ...
        names{c},kps(c),kis(c),difference);
    if c_available
        infile = [tempname '.bin']; outfile = [tempname '.bin'];
        f = fopen(infile,'wb'); fwrite(f,data.','double'); fclose(f);
        cmd = sprintf('"%s" "%s" "%s" %.17g %.17g',exe,infile,outfile,kps(c),kis(c));
        [status,message] = system(cmd); assert(status==0,'%s',message);
        f = fopen(outfile,'rb'); firmware = fread(f,[4 Inf],'double').'; fclose(f);
        delete(infile); delete(outfile);
        c_difference = max(abs(y-firmware),[],'all');
        assert(c_difference<1e-4,'Original PI_Cale.c float trace mismatch: %s',names{c});
        fprintf(fid,'  original C (float) max difference %.9g: PASSED\n',c_difference);
    else
        fprintf(fid,'  original C check SKIPPED: bundled LCC64 unavailable\n');
    end
    fprintf('%s source PI checks PASSED\n',names{c});
    clear close_bench;
end
fprintf(fid,'All three FOC PI instances PASSED. No extra Ts multiplier, no substituted PID block.\n');
end
