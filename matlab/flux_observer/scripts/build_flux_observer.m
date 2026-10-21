function model = build_flux_observer(outdir,startup_mode)
% Build a fully expanded, native-block Simulink model (no black-box S-function).
d = flux_observer_paths();
previous_filegen = Simulink.fileGenControl('getConfig');
restore_filegen = onCleanup(@()Simulink.fileGenControl('setConfig','config',previous_filegen));
Simulink.fileGenControl('set','CacheFolder',d.work,'CodeGenFolder',d.work,'createDir',true);
if nargin<1 || isempty(outdir), outdir = d.model; end
if nargin<2, startup_mode='aligned'; end
p = flux_observer_params();
[observer_input,truth] = flux_observer_demo_input(p);
s0 = flux_observer_init(p,startup_mode,observer_input.Data(1,:),0);
model = 'G474_Hybrid_Flux_Observer';
if bdIsLoaded(model), close_system(model,0); end
load_system('simulink'); build_firmware_pi_library(); new_system(model);
set_param(model,'SolverType','Fixed-step','Solver','FixedStepDiscrete', ...
    'FixedStep','p.Ts','StopTime','2','ReturnWorkspaceOutputs','on', ...
    'SaveTime','on','TimeSaveName','tout','SignalLogging','off');
ws = get_param(model,'ModelWorkspace');
assignin(ws,'p',p); assignin(ws,'observer_input',observer_input);
assignin(ws,'truth',truth);
assignin(ws,'s0',s0); assignin(ws,'startup_mode',startup_mode);
B(model,'FromWorkspace','Inputs','VariableName','observer_input','Interpolate','off', ...
    'OutputAfterFinalValue','Holding final value');
B(model,'Demux','Input split','Outputs','[2 3 2 1 1 1]');
W(model,'Inputs/1','Input split/1');

% 1. Actual firmware voltage scaling and polarity-based deadtime compensation.
s = S(model,'Deadtime and voltage',{'command_pu','phase_current','Vbus','Km'},{'voltage_V'});
B(s,'Constant','Threshold','Value','p.current_deadband');
B(s,'Constant','Negative threshold','Value','-p.current_deadband');
B(s,'RelationalOperator','Positive','Operator','>');
B(s,'RelationalOperator','Negative','Operator','<');
W(s,'phase_current/1','Positive/1'); W(s,'Threshold/1','Positive/2');
W(s,'phase_current/1','Negative/1'); W(s,'Negative threshold/1','Negative/2');
B(s,'DataTypeConversion','Positive double','OutDataTypeStr','double');
B(s,'DataTypeConversion','Negative double','OutDataTypeStr','double');
W(s,'Positive/1','Positive double/1'); W(s,'Negative/1','Negative double/1');
B(s,'Sum','Current polarity','Inputs','+-');
W(s,'Positive double/1','Current polarity/1'); W(s,'Negative double/1','Current polarity/2');
B(s,'Gain','Deadtime Clarke','Gain', ...
    'p.V_deadtime*[0.6666667 -0.33333335 -0.33333335;0 0.5773503 -0.5773503]', ...
    'Multiplication','Matrix(K*u)');
W(s,'Current polarity/1','Deadtime Clarke/1');
B(s,'Product','Bus scale','Inputs','**');
W(s,'Vbus/1','Bus scale/1'); W(s,'Km/1','Bus scale/2');
B(s,'Product','Command in volts','Inputs','**');
W(s,'command_pu/1','Command in volts/1'); W(s,'Bus scale/1','Command in volts/2');
B(s,'Sum','Corrected voltage','Inputs','++');
W(s,'Command in volts/1','Corrected voltage/1'); W(s,'Deadtime Clarke/1','Corrected voltage/2');
W(s,'Corrected voltage/1','voltage_V/1');

% 2. Current-model active flux, using previous PLL angle.
s = S(model,'Current model',{'theta_previous','Id'},{'psi_I'});
B(s,'Gain','Saliency','Gain','p.Ld-p.Lq'); W(s,'Id/1','Saliency/1');
B(s,'Constant','PM flux','Value','p.Flux'); B(s,'Sum','Active scalar','Inputs','++');
W(s,'PM flux/1','Active scalar/1'); W(s,'Saliency/1','Active scalar/2');
B(s,'Trigonometry','Cos','Operator','cos'); B(s,'Trigonometry','Sin','Operator','sin');
W(s,'theta_previous/1','Cos/1'); W(s,'theta_previous/1','Sin/1');
B(s,'Mux','Direction','Inputs','2'); W(s,'Cos/1','Direction/1'); W(s,'Sin/1','Direction/2');
B(s,'Product','Projection','Inputs','**');
W(s,'Active scalar/1','Projection/1'); W(s,'Direction/1','Projection/2'); W(s,'Projection/1','psi_I/1');

% 3. psi_V[k] = psi_s[k] - Lq*i[k].
s = S(model,'Voltage model extraction',{'psi_s_previous','current_ab'},{'psi_V'});
B(s,'Gain','Inductive flux','Gain','p.Lq'); W(s,'current_ab/1','Inductive flux/1');
B(s,'Sum','Active flux','Inputs','+-');
W(s,'psi_s_previous/1','Active flux/1'); W(s,'Inductive flux/1','Active flux/2');
W(s,'Active flux/1','psi_V/1');

% 4. Compensation integral is updated BEFORE calculating Ucomp, as in C.
s = S(model,'Flux feedback PI or SMO',{'psi_I','psi_V'},{'Ucomp'});
B(s,'Sum','Flux error','Inputs','+-'); W(s,'psi_I/1','Flux error/1'); W(s,'psi_V/1','Flux error/2');
B(s,'Demux','Alpha beta errors','Outputs','2'); W(s,'Flux error/1','Alpha beta errors/1');
B(s,'Constant','PI enable','Value','p.Comp_Mode==0');
add_block('G474_Firmware_PI/Flux_Compensation_PI',[s '/Alpha_PI']);
add_block('G474_Firmware_PI/Flux_Compensation_PI',[s '/Beta_PI']);
set_param([s '/Alpha_PI'],'Kp','p.Comp_Kp','Ki','p.Comp_Ki','Ts','p.Ts');
set_param([s '/Beta_PI'],'Kp','p.Comp_Kp','Ki','p.Comp_Ki','Ts','p.Ts');
W(s,'Alpha beta errors/1','Alpha_PI/1'); W(s,'Alpha beta errors/2','Beta_PI/1');
W(s,'PI enable/1','Alpha_PI/2'); W(s,'PI enable/1','Beta_PI/2');
B(s,'Mux','PI voltage','Inputs','2'); W(s,'Alpha_PI/1','PI voltage/1'); W(s,'Beta_PI/1','PI voltage/2');
B(s,'Gain','Boundary normalization','Gain','1/p.SMO_Boundary');
W(s,'Flux error/1','Boundary normalization/1');
B(s,'Saturation','Boundary saturation','UpperLimit','1','LowerLimit','-1');
W(s,'Boundary normalization/1','Boundary saturation/1');
B(s,'Gain','Sliding gain','Gain','p.SMO_Kslide'); W(s,'Boundary saturation/1','Sliding gain/1');
B(s,'Constant','Compensation mode','Value','p.Comp_Mode');
B(s,'Switch','Select feedback','Criteria','u2 ~= 0');
W(s,'Sliding gain/1','Select feedback/1'); W(s,'Compensation mode/1','Select feedback/2');
W(s,'PI voltage/1','Select feedback/3'); W(s,'Select feedback/1','Ucomp/1');
pack(s,'SMO_feedback',{'Boundary normalization','Boundary saturation','Sliding gain'});

% 5. Explicit Euler stator flux: old state is exposed for voltage extraction.
s = S(model,'Stator flux integrator',{'voltage_V','current_ab','Ucomp'},{'psi_s_new','psi_s_previous'});
B(s,'Gain','Resistive voltage','Gain','p.Rs'); W(s,'current_ab/1','Resistive voltage/1');
B(s,'Sum','Flux derivative','Inputs','+-+'); W(s,'voltage_V/1','Flux derivative/1');
W(s,'Resistive voltage/1','Flux derivative/2'); W(s,'Ucomp/1','Flux derivative/3');
B(s,'Gain','Euler Ts','Gain','p.Ts'); W(s,'Flux derivative/1','Euler Ts/1');
D(s,'Stator memory','s0.psi_s'); B(s,'Sum','Updated flux','Inputs','++');
W(s,'Stator memory/1','Updated flux/1'); W(s,'Euler Ts/1','Updated flux/2');
W(s,'Updated flux/1','Stator memory/1'); W(s,'Updated flux/1','psi_s_new/1');
W(s,'Stator memory/1','psi_s_previous/1');

% 6. The firmware PLL output is an ANGLE STEP. Do not multiply it by Ts.
s = S(model,'PLL',{'psi_V','theta_previous'},{'theta_new','step_rad','PLL_error'});
B(s,'Demux','Flux components','Outputs','2'); W(s,'psi_V/1','Flux components/1');
B(s,'Trigonometry','Cos','Operator','cos'); B(s,'Trigonometry','Sin','Operator','sin');
W(s,'theta_previous/1','Cos/1'); W(s,'theta_previous/1','Sin/1');
B(s,'Product','Beta cos','Inputs','**'); W(s,'Flux components/2','Beta cos/1'); W(s,'Cos/1','Beta cos/2');
B(s,'Product','Alpha sin','Inputs','**'); W(s,'Flux components/1','Alpha sin/1'); W(s,'Sin/1','Alpha sin/2');
B(s,'Sum','Phase error','Inputs','+-'); W(s,'Beta cos/1','Phase error/1'); W(s,'Alpha sin/1','Phase error/2');
W(s,'Phase error/1','PLL_error/1');
add_block('G474_Firmware_PI/PLL_PI',[s '/PLL_PI']);
set_param([s '/PLL_PI'],'Kp','p.PLL_kp','Ki','p.PLL_ki','Ts','p.Ts');
B(s,'Constant','Step limit value','Value','p.pll_step_limit');
W(s,'Phase error/1','PLL_PI/1'); W(s,'Step limit value/1','PLL_PI/2');
W(s,'PLL_PI/1','step_rad/1');
B(s,'Sum','Angle update','Inputs','++'); W(s,'theta_previous/1','Angle update/1'); W(s,'PLL_PI/1','Angle update/2');
B(s,'Math','Wrap 2pi','Operator','mod'); B(s,'Constant','Two pi','Value','2*pi');
W(s,'Angle update/1','Wrap 2pi/1'); W(s,'Two pi/1','Wrap 2pi/2'); W(s,'Wrap 2pi/1','theta_new/1');
pack(s,'Phase_detector',{'Flux components','Cos','Sin','Beta cos','Alpha sin','Phase error'});
pack(s,'Angle_update',{'Angle update','Wrap 2pi','Two pi'});

% 7. Eight-cycle speed accumulation, then 0.95/0.05 low-pass and hold.
s = S(model,'Speed filter 8 cycles',{'step_rad'},{'electrical_Hz','firmware_RPM'});
D(s,'Counter memory','0'); B(s,'Constant','One','Value','1');
B(s,'Sum','Next count','Inputs','++'); W(s,'Counter memory/1','Next count/1'); W(s,'One/1','Next count/2');
B(s,'Constant','Eight','Value','p.speed_decimation');
B(s,'RelationalOperator','Update tick','Operator','>='); W(s,'Next count/1','Update tick/1'); W(s,'Eight/1','Update tick/2');
B(s,'Constant','Zero','Value','0'); B(s,'Switch','Counter reset','Criteria','u2 ~= 0');
W(s,'Zero/1','Counter reset/1'); W(s,'Update tick/1','Counter reset/2'); W(s,'Next count/1','Counter reset/3');
W(s,'Counter reset/1','Counter memory/1');
D(s,'Step accumulator','0'); B(s,'Sum','Accumulated step','Inputs','++');
W(s,'Step accumulator/1','Accumulated step/1'); W(s,'step_rad/1','Accumulated step/2');
B(s,'Switch','Accumulator reset','Criteria','u2 ~= 0'); W(s,'Zero/1','Accumulator reset/1');
W(s,'Update tick/1','Accumulator reset/2'); W(s,'Accumulated step/1','Accumulator reset/3');
W(s,'Accumulator reset/1','Step accumulator/1');
B(s,'Gain','Step to Hz','Gain','1/(p.speed_decimation*p.Ts*2*pi)'); W(s,'Accumulated step/1','Step to Hz/1');
D(s,'Hz memory','0'); B(s,'Gain','Old weight','Gain','p.speed_filter_old'); W(s,'Hz memory/1','Old weight/1');
B(s,'Gain','New weight','Gain','p.speed_filter_new'); W(s,'Step to Hz/1','New weight/1');
B(s,'Sum','Filtered candidate','Inputs','++'); W(s,'Old weight/1','Filtered candidate/1'); W(s,'New weight/1','Filtered candidate/2');
B(s,'Switch','Update or hold','Criteria','u2 ~= 0'); W(s,'Filtered candidate/1','Update or hold/1');
W(s,'Update tick/1','Update or hold/2'); W(s,'Hz memory/1','Update or hold/3');
W(s,'Update or hold/1','Hz memory/1'); W(s,'Update or hold/1','electrical_Hz/1');
B(s,'Gain','Firmware RPM scale','Gain','60/(p.MOTOR_POLES/2)');
W(s,'Update or hold/1','Firmware RPM scale/1'); W(s,'Firmware RPM scale/1','firmware_RPM/1');

% Top-level wiring.
W(model,'Input split/1','Deadtime and voltage/1'); W(model,'Input split/2','Deadtime and voltage/2');
W(model,'Input split/5','Deadtime and voltage/3'); W(model,'Input split/6','Deadtime and voltage/4');
D(model,'Previous PLL angle','s0.theta'); W(model,'PLL/1','Previous PLL angle/1');
W(model,'Previous PLL angle/1','Current model/1'); W(model,'Input split/4','Current model/2');
W(model,'Stator flux integrator/2','Voltage model extraction/1'); W(model,'Input split/3','Voltage model extraction/2');
W(model,'Current model/1','Flux feedback PI or SMO/1'); W(model,'Voltage model extraction/1','Flux feedback PI or SMO/2');
W(model,'Deadtime and voltage/1','Stator flux integrator/1'); W(model,'Input split/3','Stator flux integrator/2');
W(model,'Flux feedback PI or SMO/1','Stator flux integrator/3');
W(model,'Voltage model extraction/1','PLL/1'); W(model,'Previous PLL angle/1','PLL/2');
W(model,'PLL/2','Speed filter 8 cycles/1');
B(model,'Mux','Results','Inputs','8');
src = {'PLL/1','Speed filter 8 cycles/1','Speed filter 8 cycles/2', ...
    'Voltage model extraction/1','Stator flux integrator/1','PLL/3','PLL/2','Deadtime and voltage/1'};
for k=1:numel(src), W(model,src{k},sprintf('Results/%d',k)); end
B(model,'ToWorkspace','Output log','VariableName','observer_y','SaveFormat','Timeseries');
W(model,'Results/1','Output log/1');
B(model,'Scope','Observer signals'); W(model,'Results/1','Observer signals/1');
% Three top-level areas; all algorithm feedback remains inside the observer.
pack(model,'01_Input',{'Inputs'});
pack(model,'02_Flux_Observer',{'Input split','Deadtime and voltage','Current model', ...
    'Voltage model extraction','Flux feedback PI or SMO','Stator flux integrator', ...
    'PLL','Previous PLL angle','Speed filter 8 cycles','Results'});
pack(model,'03_Results',{'Output log','Observer signals'});
core = [model '/02_Flux_Observer'];
pack(core,'02_Active_Flux_Models',{'Current model','Voltage model extraction'});
pack(core,'05_PLL_and_Angle',{'PLL','Previous PLL angle'});
rename(core,'Deadtime and voltage','01_Voltage_Compensation');
rename(core,'Flux feedback PI or SMO','03_Flux_Feedback');
rename(core,'Stator flux integrator','04_Stator_Integrator');
rename(core,'Speed filter 8 cycles','06_Speed_Filter');
subs = find_system(model,'BlockType','SubSystem');
for k=numel(subs):-1:1
    if strcmp(get_param(subs{k},'LinkStatus'),'resolved'), continue; end
    Simulink.BlockDiagram.arrangeSystem(subs{k});
    simple_icon(subs{k});
end
% Reflow parents after assigning readable icon sizes to every child.
for k=numel(subs):-1:1
    if ~strcmp(get_param(subs{k},'LinkStatus'),'resolved')
        Simulink.BlockDiagram.arrangeSystem(subs{k});
    end
end
Simulink.BlockDiagram.arrangeSystem(model);
set_param([model '/01_Input'],'Position',[50 150 230 240],'BackgroundColor','cyan');
set_param([model '/02_Flux_Observer'],'Position',[340 120 660 270],'BackgroundColor','lightBlue');
set_param([model '/03_Results'],'Position',[770 150 950 240],'BackgroundColor','green');
set_param([model '/01_Input'],'MaskDisplay','disp(''Input''); port_label(''output'',1,''u[10]'');');
set_param([model '/02_Flux_Observer'],'MaskDisplay','disp(''Hybrid Active Flux Observer''); port_label(''input'',1,''u[10]''); port_label(''output'',1,''y[11]'');');
set_param([model '/03_Results'],'MaskDisplay','disp(''Results''); port_label(''input'',1,''y[11]'');');
note = Simulink.Annotation(model, sprintf(['Hybrid Active Flux | source c43e794 | Ts = 50 us\n' ...
    'Rs = 0.875 ohm, Ld = Lq = 0.2 mH, PM flux = 0.00752 Wb\n' ...
    'PLL 15 Hz; compensation 5 Hz; deadtime +0.3 V\n' ...
    'Input -> Flux observer -> Results. Double-click to inspect each level.\n' ...
    'PLL output is rad/sample. RPM preserves MOTOR_POLES/2 from firmware.']));
note.Position = [50 10 950 105];
startup_note = Simulink.Annotation(model,sprintf(['Startup: %s | alignment command = 0 rad\n' ...
    'Aligned mode seeds flux only; PLL integral and speed start at zero. PI gains unchanged.'],startup_mode));
startup_note.Position = [50 290 950 340];
configure_observer_scopes(model);
set_param(model,'PostLoadFcn',sprintf('load_system(fullfile(fileparts(get_param(''%s'',''FileName'')),''G474_Firmware_PI.slx''));',model));
set_param(model,'ZoomFactor','FitSystem');
set_param(model,'SimulationCommand','update');
save_system(model,fullfile(outdir,[model '.slx']));
print(['-s' model],'-dpng','-r110',fullfile(d.results,'flux_observer_diagram.png'));
print(['-s' core],'-dpng','-r110',fullfile(d.results,'observer_subsystems.png'));
print('-sG474_Firmware_PI','-dpng','-r110',fullfile(d.results,'firmware_pi_library.png'));
print(['-s' model '/03_Results'],'-dpng','-r110',fullfile(d.results,'observer_scopes.png'));
end

function path = B(parent,kind,name,varargin)
path = [parent '/' name];
lib = struct('FromWorkspace','Sources/From Workspace','ToWorkspace','Sinks/To Workspace', ...
    'Demux','Signal Routing/Demux','Mux','Signal Routing/Mux','Constant','Sources/Constant', ...
    'Gain','Math Operations/Gain','Sum','Math Operations/Sum','Product','Math Operations/Product', ...
    'Trigonometry','Math Operations/Trigonometric Function','Math','Math Operations/Math Function', ...
    'RelationalOperator','Logic and Bit Operations/Relational Operator', ...
    'DataTypeConversion','Signal Attributes/Data Type Conversion','Switch','Signal Routing/Switch', ...
    'Saturation','Discontinuities/Saturation','Scope','Sinks/Scope');
add_block(['simulink/' lib.(kind)],path,varargin{:});
end
function s = S(parent,name,ins,outs)
s = [parent '/' name]; add_block('built-in/Subsystem',s);
for k=1:numel(ins), add_block('built-in/Inport',[s '/' ins{k}],'Port',num2str(k)); end
for k=1:numel(outs), add_block('built-in/Outport',[s '/' outs{k}],'Port',num2str(k)); end
end
function D(s,name,ic)
add_block('simulink/Discrete/Unit Delay',[s '/' name],'SampleTime','p.Ts','InitialCondition',ic);
end
function path = pack(parent,name,names)
handles = zeros(1,numel(names));
for k=1:numel(names), handles(k) = get_param([parent '/' names{k}],'Handle'); end
Simulink.BlockDiagram.createSubsystem(handles,'Name',name);
path = [parent '/' name];
end
function rename(parent,old,new)
set_param([parent '/' old],'Name',new);
end
function simple_icon(path)
% Explicit icons keep internal wiring from obscuring the subsystem labels.
if ~strcmp(get_param(path,'Mask'),'on')
    mask = Simulink.Mask.create(path);
    mask.Display = '';
    ins = find_system(path,'SearchDepth',1,'BlockType','Inport');
    outs = find_system(path,'SearchDepth',1,'BlockType','Outport');
    for k=1:numel(ins)
        mask.Display = [mask.Display sprintf('port_label(''input'',%s,''%s'');', ...
            get_param(ins{k},'Port'),get_param(ins{k},'Name'))];
    end
    for k=1:numel(outs)
        mask.Display = [mask.Display sprintf('port_label(''output'',%s,''%s'');', ...
            get_param(outs{k},'Port'),get_param(outs{k},'Name'))];
    end
    set_param(path,'OpenFcn','open_system(gcb,''force'')','BackgroundColor','lightBlue');
    pos = get_param(path,'Position');
    height = max(110,32*(1+max(numel(ins),numel(outs))));
    set_param(path,'Position',[pos(1:2) pos(1)+320 pos(2)+height]);
end
end
function W(s,src,dst)
add_line(s,src,dst,'autorouting','on');
end
