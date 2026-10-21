function model=build_mode2()
% Build the separate closed-loop experiment using embedded native firmware PIs.
d=mode2_paths(); [p,q,s0]=mode2_params();
old=Simulink.fileGenControl('getConfig'); restore=onCleanup(@()Simulink.fileGenControl('setConfig','config',old));
Simulink.fileGenControl('set','CacheFolder',d.work,'CodeGenFolder',d.work,'createDir',true);
source=fullfile(fileparts(d.root),'flux_observer','model');
load_system(fullfile(source,'G474_Firmware_PI.slx'));
load_system(fullfile(source,'G474_Hybrid_Flux_Observer.slx'));
model='G474_Mode2_Direct_Start_Repair';
if bdIsLoaded(model), close_system(model,0); end
new_system(model); load_system('simulink');
set_param(model,'SolverType','Fixed-step','Solver','FixedStepDiscrete', ...
    'FixedStep','p.Ts','StopTime','q.stop_time','ReturnWorkspaceOutputs','on', ...
    'SaveTime','on','TimeSaveName','tout','SignalLogging','off');
table_source=fileread(fullfile(d.root,'scripts','reference','Common_Math.c'));
table_source=regexp(table_source,'const float fSin\[1025\][\s\S]*?\{([\s\S]*?)\};','tokens','once');
table_source=regexprep(table_source{1},'//[^\r\n]*','');
fSin=double(single(sscanf(strrep(table_source,',',' '),'%f')));
assert(numel(fSin)==1025,'Source sine table must have 1025 entries.');
ws=get_param(model,'ModelWorkspace');
assignin(ws,'p',p); assignin(ws,'q',q); assignin(ws,'s0',s0); assignin(ws,'fSin',fSin);

% Firmware mode=2 has no VF or preposition phase.
s=sub(model,'01_Mode2_Command',{}, {'target_rpm','mode'});
b(s,'simulink/Sources/Constant','Initial_200rpm','Value','q.target_rpm');
b(s,'simulink/Sources/Step','Switch_to_400rpm','Time','q.switch_1','Before','0','After','q.second_rpm-q.target_rpm','SampleTime','p.Ts');
b(s,'simulink/Sources/Step','Switch_to_1000rpm','Time','q.switch_2','Before','0','After','q.third_rpm-q.second_rpm','SampleTime','p.Ts');
b(s,'simulink/Math Operations/Sum','Speed_schedule','Inputs','+++');
w(s,'Initial_200rpm/1','Speed_schedule/1'); w(s,'Switch_to_400rpm/1','Speed_schedule/2'); w(s,'Switch_to_1000rpm/1','Speed_schedule/3');
w(s,'Speed_schedule/1','target_rpm/1'); constant(s,'MODE_SENSORLESS_2','2','mode/1');

% New independent copy; all linked PI blocks detached for a portable .slx.
core=[model '/03_Hybrid_Flux_Observer'];
add_block('G474_Hybrid_Flux_Observer/02_Flux_Observer',core);
detach(core);
% Repair boundary: observer receives already reconstructed applied volts.
voltage=[core '/01_Voltage_Compensation'];
vl=get_param(voltage,'LineHandles');
old_lines=unique([vl.Inport(:);vl.Outport(:)]);
for j=1:numel(old_lines), if old_lines(j)~=-1, delete_line(old_lines(j)); end, end
Simulink.SubSystem.deleteContents(voltage);
for k=1:4, b(voltage,'built-in/Inport',sprintf('input%d',k),'Port',num2str(k)); end
b(voltage,'built-in/Outport','Applied_voltage_V');
w(voltage,'input1/1','Applied_voltage_V/1');
for k=2:4
    b(voltage,'simulink/Sinks/Terminator',sprintf('Unused_legacy_port%d',k));
    w(voltage,sprintf('input%d/1',k),sprintf('Unused_legacy_port%d/1',k));
end
note(voltage,'Applied VOLTS reconstructed upstream from PWM and measured current.\nNo second bus scaling or deadtime correction. PI equations and gains remain source values.');
Simulink.BlockDiagram.arrangeSystem(voltage);
% Deleting subsystem ports disconnects outer lines: reconnect the original frame.
vp=get_param(voltage,'PortHandles');
wire_sources={'Input split/1','Input split/2','Input split/5','Input split/6'};
for k=1:4, w(core,wire_sources{k},sprintf('01_Voltage_Compensation/%d',k)); end
w(core,'01_Voltage_Compensation/1','04_Stator_Integrator/1');
% The logged voltage was also connected through the observer result mux.
w(core,'01_Voltage_Compensation/1','Results/8');
set_param(core,'MaskDisplay','disp(''Hybrid Flux / PLL''); port_label(''input'',1,''frame[10]''); port_label(''output'',1,''observer[11]'');');
s=sub(model,'02_PWM_Voltage_and_Current_Sampling',{'phase_i','previous_ab','previous_dq','previous_command'},{'observer_frame'});
mf(s,'01_PWM_CCR_Applied_Voltage','reconstruct_voltage.m',{'q'});
w(s,'previous_command/1','01_PWM_CCR_Applied_Voltage/1'); w(s,'previous_ab/1','01_PWM_CCR_Applied_Voltage/2');
b(s,'simulink/Math Operations/Gain','02_Fresh_Clarke_Current','Gain','[1 0 0;0.577350269 1.154700538 0]','Multiplication','Matrix(K*u)');
w(s,'phase_i/1','02_Fresh_Clarke_Current/1');
b(s,'simulink/Signal Routing/Selector','Previous_Id','NumberOfDimensions','1','IndexOptionArray',{'Index vector (dialog)'},'Indices','1','InputPortWidth','2');
w(s,'previous_dq/1','Previous_Id/1');
b(s,'simulink/Signal Routing/Mux','03_Observer_Frame_Volts_and_Amps','Inputs','6');
w(s,'01_PWM_CCR_Applied_Voltage/1','03_Observer_Frame_Volts_and_Amps/1');
w(s,'phase_i/1','03_Observer_Frame_Volts_and_Amps/2');
w(s,'02_Fresh_Clarke_Current/1','03_Observer_Frame_Volts_and_Amps/3');
w(s,'Previous_Id/1','03_Observer_Frame_Volts_and_Amps/4');
constant(s,'Bus_V','q.Vbus','03_Observer_Frame_Volts_and_Amps/5');
constant(s,'Bus_Km','min(max(12/q.Vbus,0.7),1.3)','03_Observer_Frame_Volts_and_Amps/6');
w(s,'03_Observer_Frame_Volts_and_Amps/1','observer_frame/1');
note(s,'REPAIRED: reconstruct previous applied voltage from command / CCR, estimate deadtime ONCE.\nObserver uses CURRENT Clarke samples. No true angle, speed or physical plant voltage input.');

s=sub(model,'04_Clarke_Park',{'phase_i','theta_est'},{'Idq_measured','current_ab','cos_sin'});
mf(s,'Source_lookup_and_transform','firmware_park.m',{'fSin'});
w(s,'phase_i/1','Source_lookup_and_transform/1'); w(s,'theta_est/1','Source_lookup_and_transform/2');
for k=1:3, w(s,sprintf('Source_lookup_and_transform/%d',k),sprintf('%s/1',get_out(s,k))); end

s=sub(model,'05_Speed_Ramp_and_PI',{'target_rpm','estimated_rpm'},{'Iq_reference','ramped_rpm'});
b(s,'simulink/Sources/Digital Clock','Warmup_elapsed_time','SampleTime','p.Ts');
b(s,'simulink/Sources/Constant','Warmup_duration_s','Value','q.reference_start_delay_s');
b(s,'simulink/Logic and Bit Operations/Relational Operator','Warmup_time_elapsed','Operator','>=');
w(s,'Warmup_elapsed_time/1','Warmup_time_elapsed/1'); w(s,'Warmup_duration_s/1','Warmup_time_elapsed/2');
b(s,'simulink/Sources/Constant','Zero_reference_during_warmup','Value','0');
b(s,'simulink/Signal Routing/Switch','Reference_release_after_warmup','Criteria','u2 ~= 0');
w(s,'target_rpm/1','Reference_release_after_warmup/1'); w(s,'Warmup_time_elapsed/1','Reference_release_after_warmup/2');
w(s,'Zero_reference_during_warmup/1','Reference_release_after_warmup/3');
reference_filter(s,'Reference_filter_1','Reference_release_after_warmup/1');
reference_filter(s,'Reference_filter_2','Reference_filter_1/1');
delay(s,'Previous_ramped_rpm','0');
b(s,'simulink/Math Operations/Sum','Ramp_error','Inputs','+-');
w(s,'Reference_filter_2/1','Ramp_error/1'); w(s,'Previous_ramped_rpm/1','Ramp_error/2');
b(s,'simulink/Discontinuities/Saturation','Source_ramp_1000rpm_per_second','UpperLimit','q.ramp_step','LowerLimit','-q.ramp_step');
w(s,'Ramp_error/1','Source_ramp_1000rpm_per_second/1');
b(s,'simulink/Math Operations/Sum','Ramp_update','Inputs','++');
w(s,'Previous_ramped_rpm/1','Ramp_update/1'); w(s,'Source_ramp_1000rpm_per_second/1','Ramp_update/2');
w(s,'Ramp_update/1','Previous_ramped_rpm/1'); w(s,'Ramp_update/1','ramped_rpm/1');
pi_block(s,'Speed_PI','FOC_Speed_PI');
w(s,'Ramp_update/1','Speed_PI/1'); w(s,'estimated_rpm/1','Speed_PI/2');
set_param([s '/Speed_PI'],'Kp','q.speed_kp','Ki','q.speed_ki');
constant(s,'Upper_12A','q.iq_limit','Speed_PI/3'); constant(s,'Lower_minus12A','-q.iq_limit','Speed_PI/4');
filtered(s,'Speed_OutF','Speed_PI/1');
b(s,'simulink/Math Operations/Gain','Iq_Torque_Polarity_plus1','Gain','q.speed_iq_sign');
w(s,'Speed_OutF/1','Iq_Torque_Polarity_plus1/1'); w(s,'Iq_Torque_Polarity_plus1/1','Iq_reference/1');
note(s,'MODE=2 throughout: zero reference for 150 ms while observer states naturally converge.\nNo alignment current, VF, true angle or flux preset. Then 2-pole 2-Hz shaping and source slew limiter.\nPI gains/anti-windup equations, PLL and speed filter remain unchanged.');
group(s,'01_Observer_Warmup_and_Command_Shaping',{'Warmup_elapsed_time','Warmup_duration_s','Warmup_time_elapsed','Zero_reference_during_warmup','Reference_release_after_warmup', ...
    'Reference_filter_1_memory','Reference_filter_1_error','Reference_filter_1_alpha','Reference_filter_1', ...
    'Reference_filter_2_memory','Reference_filter_2_error','Reference_filter_2_alpha','Reference_filter_2'},'Zero-reference warmup 150 ms\nThen two 2-Hz reference filters');
group(s,'02_Speed_Slew_Limiter',{'Previous_ramped_rpm','Ramp_error','Source_ramp_1000rpm_per_second','Ramp_update'},'Speed slew limiter\n1000 rpm per second');
group(s,'03_Speed_PI_Output_and_Polarity',{'Speed_PI','Upper_12A','Lower_minus12A','Speed_OutF_previous','Speed_OutF_old_09','Speed_OutF_new_01','Speed_OutF','Iq_Torque_Polarity_plus1'},'Source speed PI / OutF\nIq limit 12 A / polarity +1');

s=sub(model,'06_Id_Iq_PI_and_Voltage_Circle',{'Iq_reference','Idq_measured'},{'Vdq_pu'});
b(s,'simulink/Signal Routing/Demux','Split_Id_Iq','Outputs','2'); w(s,'Idq_measured/1','Split_Id_Iq/1');
pi_block(s,'Id_PI','FOC_Id_PI'); pi_block(s,'Iq_PI','FOC_Iq_PI');
set_param([s '/Id_PI'],'Kp','q.current_kp','Ki','q.current_ki');
set_param([s '/Iq_PI'],'Kp','q.current_kp','Ki','q.current_ki');
constant(s,'Id_reference_zero','0','Id_PI/1'); w(s,'Split_Id_Iq/1','Id_PI/2');
constant(s,'Bus_voltage_limit','min(max(q.Vbus/12,0),0.95)','Id_PI/3');
constant(s,'Negative_bus_limit','-min(max(q.Vbus/12,0),0.95)','Id_PI/4');
filtered(s,'Id_OutF','Id_PI/1');
b(s,'simulink/Math Operations/Math Function','Vd_squared','Operator','square'); w(s,'Id_OutF/1','Vd_squared/1');
constant(s,'Voltage_circle_squared','min(max(q.Vbus/12,0),0.95)^2','Remaining_voltage/1',false);
b(s,'simulink/Math Operations/Sum','Remaining_voltage','Inputs','+-');
w(s,'Voltage_circle_squared/1','Remaining_voltage/1'); w(s,'Vd_squared/1','Remaining_voltage/2');
b(s,'simulink/Discontinuities/Saturation','Nonnegative_margin','UpperLimit','inf','LowerLimit','0'); w(s,'Remaining_voltage/1','Nonnegative_margin/1');
b(s,'simulink/Math Operations/Math Function','Q_voltage_limit','Operator','sqrt'); w(s,'Nonnegative_margin/1','Q_voltage_limit/1');
b(s,'simulink/Math Operations/Gain','Negative_Q_limit','Gain','-1'); w(s,'Q_voltage_limit/1','Negative_Q_limit/1');
w(s,'Iq_reference/1','Iq_PI/1'); w(s,'Split_Id_Iq/2','Iq_PI/2');
w(s,'Q_voltage_limit/1','Iq_PI/3'); w(s,'Negative_Q_limit/1','Iq_PI/4');
filtered(s,'Iq_OutF','Iq_PI/1');
b(s,'simulink/Signal Routing/Mux','Vd_Vq','Inputs','2'); w(s,'Id_OutF/1','Vd_Vq/1'); w(s,'Iq_OutF/1','Vd_Vq/2'); w(s,'Vd_Vq/1','Vdq_pu/1');
group(s,'01_Id_PI_and_Filter',{'Id_PI','Id_reference_zero','Bus_voltage_limit','Negative_bus_limit','Id_OutF_previous','Id_OutF_old_09','Id_OutF_new_01','Id_OutF'},'d-axis source PI\nId = 0 A / OutF');
group(s,'02_Voltage_Circle',{'Vd_squared','Voltage_circle_squared','Remaining_voltage','Nonnegative_margin','Q_voltage_limit','Negative_Q_limit'},'Voltage circle\nsqrt(limit^2 - Vd^2)');
group(s,'03_Iq_PI_and_Filter',{'Iq_PI','Iq_OutF_previous','Iq_OutF_old_09','Iq_OutF_new_01','Iq_OutF'},'q-axis source PI\nDynamic voltage limits / OutF');

s=sub(model,'07_InversePark_SVPWM_Inverter',{'Vdq_pu','cos_sin','phase_i','estimated_rpm'},{'command_pu','actual_voltage_ab','CCR_abc'});
mf(s,'Inverse_Park','firmware_inverse_park.m',{}); mf(s,'Source_SVPWM_and_average_bridge','firmware_svpwm.m',{'q'});
mf(s,'Deadtime_Voltage_Feedforward','deadtime_feedforward.m',{'q'});
w(s,'Vdq_pu/1','Inverse_Park/1'); w(s,'cos_sin/1','Inverse_Park/2');
w(s,'Inverse_Park/1','Deadtime_Voltage_Feedforward/1'); w(s,'phase_i/1','Deadtime_Voltage_Feedforward/2');
b(s,'simulink/Sources/Digital Clock','Mode2_elapsed_time','SampleTime','p.Ts');
w(s,'estimated_rpm/1','Deadtime_Voltage_Feedforward/3'); w(s,'Mode2_elapsed_time/1','Deadtime_Voltage_Feedforward/4');
w(s,'Deadtime_Voltage_Feedforward/1','command_pu/1'); w(s,'Deadtime_Voltage_Feedforward/1','Source_SVPWM_and_average_bridge/1'); w(s,'phase_i/1','Source_SVPWM_and_average_bridge/2');
w(s,'Source_SVPWM_and_average_bridge/1','actual_voltage_ab/1'); w(s,'Source_SVPWM_and_average_bridge/2','CCR_abc/1');
note(s,'Inverter deadtime feedforward: measured current polarity -> voltage-loss estimate -> PWM command.\nSource SVPWM sector / CCR truncation / 0..3999 clamp retained.\nPhysical voltage is independently reconstructed from duty ratios and physical deadtime.');

s=sub(model,'08_PMSM_Electrical_Mechanical_Plant',{'voltage_ab'},{'phase_i','physical_ab','actual_rpm','actual_theta','torque'});
delay(s,'Physical_state_id_iq_wm_theta','q.initial_state');
mf(s,'Electrical_and_mechanical_RK4','pmsm_rk4.m',{'q'}); mf(s,'Physical_measurements','phase_measurements.m',{'q'});
w(s,'Physical_state_id_iq_wm_theta/1','Electrical_and_mechanical_RK4/1'); w(s,'voltage_ab/1','Electrical_and_mechanical_RK4/2');
w(s,'Electrical_and_mechanical_RK4/1','Physical_state_id_iq_wm_theta/1'); w(s,'Physical_state_id_iq_wm_theta/1','Physical_measurements/1');
for k=1:5, w(s,sprintf('Physical_measurements/%d',k),sprintf('%s/1',get_out(s,k))); end
note(s,'True angle/speed are used only by the physical plant and result plots.\nJ=2e-5 kg*m^2, B=1e-5 N*m*s/rad: assumed.\nDefault load=0.005*tanh(wm/1) Nm, opposing motion; zero scale selects fixed external torque.\nInitial speed/current=0; initial electrical angle=30 deg; observer state=0.');

% Explicit firmware history: observer sees previous control command / Clarke / Park.
delay(model,'Previous_command_pu','[0;0]'); delay(model,'Previous_Clarke_ab','[0;0]'); delay(model,'Previous_Park_dq','[0;0]');
w(model,'07_InversePark_SVPWM_Inverter/1','Previous_command_pu/1');
w(model,'04_Clarke_Park/2','Previous_Clarke_ab/1'); w(model,'04_Clarke_Park/1','Previous_Park_dq/1');
w(model,'08_PMSM_Electrical_Mechanical_Plant/1','02_PWM_Voltage_and_Current_Sampling/1');
w(model,'Previous_Clarke_ab/1','02_PWM_Voltage_and_Current_Sampling/2'); w(model,'Previous_Park_dq/1','02_PWM_Voltage_and_Current_Sampling/3');
w(model,'Previous_command_pu/1','02_PWM_Voltage_and_Current_Sampling/4'); w(model,'02_PWM_Voltage_and_Current_Sampling/1','03_Hybrid_Flux_Observer/1');
b(model,'simulink/Signal Routing/Demux','Observer_split','Outputs','11'); w(model,'03_Hybrid_Flux_Observer/1','Observer_split/1');
w(model,'08_PMSM_Electrical_Mechanical_Plant/1','04_Clarke_Park/1'); w(model,'Observer_split/1','04_Clarke_Park/2');
w(model,'01_Mode2_Command/1','05_Speed_Ramp_and_PI/1'); w(model,'Observer_split/3','05_Speed_Ramp_and_PI/2');
w(model,'05_Speed_Ramp_and_PI/1','06_Id_Iq_PI_and_Voltage_Circle/1'); w(model,'04_Clarke_Park/1','06_Id_Iq_PI_and_Voltage_Circle/2');
w(model,'06_Id_Iq_PI_and_Voltage_Circle/1','07_InversePark_SVPWM_Inverter/1');
w(model,'04_Clarke_Park/3','07_InversePark_SVPWM_Inverter/2'); w(model,'08_PMSM_Electrical_Mechanical_Plant/1','07_InversePark_SVPWM_Inverter/3');
w(model,'Observer_split/3','07_InversePark_SVPWM_Inverter/4');
w(model,'07_InversePark_SVPWM_Inverter/2','08_PMSM_Electrical_Mechanical_Plant/1');

% Flat, documented log for independent regression and exactly eleven scopes.
b(model,'simulink/Signal Routing/Mux','Simulation_log_32','Inputs','15');
src={'03_Hybrid_Flux_Observer/1','08_PMSM_Electrical_Mechanical_Plant/3', ...
    '08_PMSM_Electrical_Mechanical_Plant/4','08_PMSM_Electrical_Mechanical_Plant/5', ...
    '08_PMSM_Electrical_Mechanical_Plant/1','04_Clarke_Park/1','06_Id_Iq_PI_and_Voltage_Circle/1', ...
    '05_Speed_Ramp_and_PI/1','05_Speed_Ramp_and_PI/2','07_InversePark_SVPWM_Inverter/1', ...
    '07_InversePark_SVPWM_Inverter/2','07_InversePark_SVPWM_Inverter/3','01_Mode2_Command/1','01_Mode2_Command/2','04_Clarke_Park/2'};
% Last two entries are current Clarke ab, useful for observer-call-order checks.
set_param([model '/Simulation_log_32'],'Name','Simulation_log_34');
for k=1:numel(src), w(model,src{k},sprintf('Simulation_log_34/%d',k)); end
s=sub(model,'09_Results_11_Scopes',{'signals34'},{});
make_results(s); w(model,'Simulation_log_34/1','09_Results_11_Scopes/1');

subs=find_system(model,'BlockType','SubSystem');
for k=numel(subs):-1:1
    if contains(subs{k},'/03_Hybrid_Flux_Observer/'), continue; end
    if contains(subs{k},'/09_Results_11_Scopes'), continue; end
    if strcmp(get_param(subs{k},'SFBlockType'),'MATLAB Function'), continue; end
    try Simulink.BlockDiagram.arrangeSystem(subs{k}); catch, end
end
% Local Goto/From routes keep the top-level wiring readable.
tidy_control_views(model);
route_top(model);
pack_support(model);
positions={[300 200 580 340],[300 1100 600 1360],[1100 1100 1400 1300], ...
    [300 650 600 880],[1100 200 1400 380],[2000 200 2330 450], ...
    [2000 650 2330 880],[1100 650 1430 900],[2000 1100 2330 1300]};
names={'01_Mode2_Command','02_PWM_Voltage_and_Current_Sampling','03_Hybrid_Flux_Observer','04_Clarke_Park', ...
    '05_Speed_Ramp_and_PI','06_Id_Iq_PI_and_Voltage_Circle','07_InversePark_SVPWM_Inverter', ...
    '08_PMSM_Electrical_Mechanical_Plant','09_Results_and_Logging'};
captions={'MODE = 2\n200 / 400 / 1000 rpm','Applied PWM voltage / fresh current\nReconstruction / sampling', ...
    'Hybrid Flux Observer\nFlux PI + PLL', 'Clarke / Park\nFirmware sine table', ...
    'Observer warmup / ramp / source PI\n150 ms zero reference / 2 Hz shaping', 'Id / Iq source PI\nVoltage circle / OutF', ...
    'Inverse Park / SVPWM\nAverage physical bridge', 'Physical PMSM\nElectrical + mechanical', ...
    '11 labelled scopes\nActual / estimated results'};
for k=1:9
    set_param([model '/' names{k}],'Position',positions{k},'BackgroundColor','lightBlue');
    icon([model '/' names{k}],captions{k});
end
set_param([model '/' names{8}],'BackgroundColor','orange'); set_param([model '/' names{9}],'BackgroundColor','green');
set_param([model '/10_Firmware_Cycle_Memory'],'Position',[300 1530 620 1670]);
set_param([model '/11_Observer_Output_Decode'],'Position',[1100 1530 1420 1670]);
icon([model '/10_Firmware_Cycle_Memory'],'Previous PWM / Clarke / Park\nOne-cycle source history');
icon([model '/11_Observer_Output_Decode'],'Observer signal decode\nEstimated angle / speed');
position_tags(model);
note(model,['MODE=2 REPAIRED DIRECT START | 20 kHz | 12 V' newline ...
    'Target: 200 -> 400 -> 1000 rpm at 0 / 0.8 / 1.6 s; firmware ramp=1000 rpm/s.' newline ...
    'No preposition, VF, flux seeding, speed preload or true-angle feedback.' newline ...
    'Repairs: Iq sign +1; PWM/CCR reconstruction; fresh current; 2-Hz command shaping; gated deadtime feedforward.' newline ...
    'Physical initial angle: random 0..360 deg each run; observer starts at zero.' newline ...
    'Orange: physical PMSM. Double-click blue subsystems to inspect firmware equations.']);
set_param(model,'ZoomFactor','FitSystem');
set_param(model,'InitFcn',[ ...
    'm2_project_root=fileparts(fileparts(get_param(bdroot,''FileName'')));' ...
    'Simulink.fileGenControl(''set'',''CacheFolder'',fullfile(m2_project_root,''work''),' ...
    '''CodeGenFolder'',fullfile(m2_project_root,''work''),''createDir'',true);' ...
    'm2_ws=get_param(bdroot,''ModelWorkspace'');m2_q=getVariable(m2_ws,''q'');' ...
    'if isfield(m2_q,''random_initial_angle'') && m2_q.random_initial_angle,' ...
    'm2_q.initial_theta_e=2*pi*rand();end;' ...
    'm2_q.initial_state=[0;0;m2_q.initial_rpm*2*pi/60;m2_q.initial_theta_e];' ...
    'assignin(m2_ws,''q'',m2_q);' ...
    'fprintf(''MODE2 physical initial electrical angle: %.9f deg (random=%d)\n'',' ...
    'm2_q.initial_theta_e*180/pi,m2_q.random_initial_angle);']);
set_param(model,'SimulationCommand','update');
save_system(model,fullfile(d.model,[model '.slx']));
print(['-s' model],'-dpng','-r120',fullfile(d.results,'model_diagram.png'));
print(['-s' model '/09_Results_and_Logging/09_Results_11_Scopes'],'-dpng','-r100',fullfile(d.results,'scopes_11.png'));
print(['-s' model '/05_Speed_Ramp_and_PI'],'-dpng','-r110',fullfile(d.results,'speed_control_modules.png'));
print(['-s' model '/06_Id_Iq_PI_and_Voltage_Circle'],'-dpng','-r110',fullfile(d.results,'current_control_modules.png'));
print(['-s' model '/02_PWM_Voltage_and_Current_Sampling'],'-dpng','-r110',fullfile(d.results,'sampling_voltage_modules.png'));
fprintf('MODE2_MODEL_BUILT\n');
end

function path=b(s,lib,name,varargin)
path=[s '/' name]; add_block(lib,path,varargin{:});
end
function w(s,a,z)
parts=strsplit(z,'/'); dest=[s '/' strjoin(parts(1:end-1),'/')];
ph=get_param(dest,'PortHandles');
line=get_param(ph.Inport(str2double(parts{end})),'Line');
if line~=-1 && get_param(line,'SrcPortHandle')==-1, delete_line(line); line=-1; end
if line==-1
    add_line(s,a,z,'autorouting','on');
end
end
function s=sub(parent,name,ins,outs)
s=[parent '/' name]; add_block('built-in/Subsystem',s);
for k=1:numel(ins), b(s,'built-in/Inport',ins{k},'Port',num2str(k)); end
for k=1:numel(outs), b(s,'built-in/Outport',outs{k},'Port',num2str(k)); end
end
function name=get_in(s,k)
list=find_system(s,'SearchDepth',1,'BlockType','Inport');
for j=1:numel(list), if str2double(get_param(list{j},'Port'))==k, name=get_param(list{j},'Name'); return; end, end
end
function name=get_out(s,k)
list=find_system(s,'SearchDepth',1,'BlockType','Outport');
for j=1:numel(list), if str2double(get_param(list{j},'Port'))==k, name=get_param(list{j},'Name'); return; end, end
end
function delay(s,name,ic)
b(s,'simulink/Discrete/Unit Delay',name,'SampleTime','p.Ts','InitialCondition',ic);
end
function constant(s,name,value,dest,connect)
if nargin<5, connect=true; end
b(s,'simulink/Sources/Constant',name,'Value',value);
if connect, w(s,[name '/1'],dest); end
end
function pi_block(s,name,source)
b(s,['G474_Firmware_PI/' source],name,'Ts','p.Ts'); set_param([s '/' name],'LinkStatus','none');
end
function filtered(s,name,source)
% Explicit updated-state output, exactly the C OutF assignment order.
delay(s,[name '_previous'],'0');
b(s,'simulink/Math Operations/Gain',[name '_old_09'],'Gain','0.9');
b(s,'simulink/Math Operations/Gain',[name '_new_01'],'Gain','0.1');
b(s,'simulink/Math Operations/Sum',name,'Inputs','++');
w(s,[name '_previous/1'],[name '_old_09/1']); w(s,source,[name '_new_01/1']);
w(s,[name '_old_09/1'],[name '/1']); w(s,[name '_new_01/1'],[name '/2']); w(s,[name '/1'],[name '_previous/1']);
end
function reference_filter(s,name,source)
delay(s,[name '_memory'],'0');
b(s,'simulink/Math Operations/Sum',[name '_error'],'Inputs','+-');
w(s,source,[name '_error/1']); w(s,[name '_memory/1'],[name '_error/2']);
b(s,'simulink/Math Operations/Gain',[name '_alpha'],'Gain','1-exp(-2*pi*q.reference_filter_hz*p.Ts)');
w(s,[name '_error/1'],[name '_alpha/1']);
b(s,'simulink/Math Operations/Sum',name,'Inputs','++');
w(s,[name '_memory/1'],[name '/1']); w(s,[name '_alpha/1'],[name '/2']);
w(s,[name '/1'],[name '_memory/1']);
end
function mf(s,name,file,params)
path=b(s,'simulink/User-Defined Functions/MATLAB Function',name);
rt=sfroot; chart=rt.find('-isa','Stateflow.EMChart','Path',path);
chart.Script=fileread(fullfile(fileparts(mfilename('fullpath')),file));
if strcmp(file,'reconstruct_voltage.m')
    chart.Script=[chart.Script newline fileread(fullfile(fileparts(mfilename('fullpath')),'firmware_svpwm.m'))];
end
for k=1:numel(params)
    datum=chart.find('-isa','Stateflow.Data','Name',params{k});
    if isempty(datum), datum=Stateflow.Data(chart); datum.Name=params{k}; end
    datum.Scope='Parameter';
end
switch file
    case 'firmware_inverse_park.m', dims={'vdq','[2 1]';'cs','[2 1]';'command','[2 1]'};
    case 'deadtime_feedforward.m', dims={'desired_command','[2 1]';'phase_i','[3 1]';'estimated_rpm','1';'time','1';'applied_command','[2 1]'};
    case 'firmware_svpwm.m', dims={'command','[2 1]';'iabc','[3 1]';'voltage','[2 1]';'ccr','[3 1]'};
    case 'reconstruct_voltage.m', dims={'old_command','[2 1]';'old_ab','[2 1]';'voltage','[2 1]'};
    case 'firmware_park.m', dims={'iabc','[3 1]';'theta','1';'dq','[2 1]';'iab','[2 1]';'cs','[2 1]'};
    case 'pmsm_rk4.m', dims={'x','[4 1]';'v','[2 1]';'xn','[4 1]'};
    case 'phase_measurements.m', dims={'x','[4 1]';'iabc','[3 1]';'iab','[2 1]';'rpm','1';'theta','1';'torque','1'};
    otherwise, dims=cell(0,2);
end
for k=1:size(dims,1)
    datum=chart.find('-isa','Stateflow.Data','Name',dims{k,1});
    if ~isempty(datum), datum.Props.Array.Size=dims{k,2}; end
end
end
function detach(s)
blocks=find_system(s,'LookUnderMasks','all','FollowLinks','on','Type','Block');
for k=1:numel(blocks)
    if strcmp(get_param(blocks{k},'LinkStatus'),'resolved'), set_param(blocks{k},'LinkStatus','none'); end
end
end
function note(s,text)
a=Simulink.Annotation(s,strrep(text,'\n',newline)); a.Position=[25 -130 1450 -10];
end
function make_results(s)
d=mode2_paths(); %#ok<NASGU>
set_param([s '/signals34'],'Position',[20 -85 45 -65]);
b(s,'simulink/Signal Routing/Goto','Scope_data_publisher','GotoTag','m2_scope_data','TagVisibility','local','Position',[100 -90 250 -60]);
w(s,'signals34/1','Scope_data_publisher/1');
b(s,'simulink/Signal Routing/From','Complete_log_tap','GotoTag','m2_scope_data','ShowName','off','Position',[450 -90 600 -60]);
b(s,'simulink/Sinks/To Workspace','Complete_log','VariableName','mode2_y','SaveFormat','Timeseries','MaxDataPoints','inf','Position',[700 -90 830 -60]);
w(s,'Complete_log_tap/1','Complete_log/1');
titles={'01_转速给定_斜坡_实际_估算_rpm','02_实际与估算电角度_rad', ...
    '03_Iq给定与反馈_A','04_Id反馈_A','05_dq电压指令_pu', ...
    '06_定子磁链alpha_beta_Wb','07_有效磁链alpha_beta_Wb','08_PLL误差_Wb', ...
    '09_电磁转矩_Nm','10_实际与观测器输入电压_V','11_三相采样电流_A'};
indices={[31 23 12 3],[13 1],[22 19],18,[20 21],[6 7],[4 5],8,14,[26 27 10 11],[15 16 17]};
labels={{'Target_rpm','Ramped_rpm','Actual_rpm','Estimated_rpm'}, {'Actual_theta_rad','Estimated_theta_rad'}, ...
    {'Iq_ref_A','Iq_measured_A'},{'Id_measured_A'},{'Vd_pu','Vq_pu'}, ...
    {'Stator_alpha_Wb','Stator_beta_Wb'},{'Active_alpha_Wb','Active_beta_Wb'}, ...
    {'PLL_error_Wb'},{'Torque_Nm'}, {'Actual_alpha_V','Actual_beta_V','Observer_alpha_V','Observer_beta_V'}, ...
    {'Iu_A','Iv_A','Iw_A'}};
for k=1:11
    x=40+mod(k-1,3)*620; y=40+floor((k-1)/3)*250;
    tap=sprintf('Scope%02d_data',k);
    b(s,'simulink/Signal Routing/From',tap,'GotoTag','m2_scope_data','ShowName','off','Position',[x y+55 x+120 y+80]);
    scope=b(s,'simulink/Sinks/Scope',titles{k},'NumInputPorts',num2str(numel(indices{k})),'Position',[x+380 y+25 x+455 y+100]);
    for j=1:numel(indices{k})
        sel=sprintf('Scope%02d_signal%d',k,j);
        b(s,'simulink/Signal Routing/Selector',sel,'NumberOfDimensions','1','IndexOptionArray',{'Index vector (dialog)'}, ...
            'Indices',num2str(indices{k}(j)),'InputPortWidth','34','ShowName','off', ...
            'Position',[x+180 y+10+(j-1)*40 x+240 y+30+(j-1)*40]);
        w(s,[tap '/1'],[sel '/1']); line=add_line(s,[sel '/1'],sprintf('%s/%d',titles{k},j),'autorouting','on');
        set_param(line,'Name',labels{k}{j});
    end
    cfg=get_param(scope,'ScopeConfiguration'); cfg.ShowLegend=true; cfg.TimeSpan='4'; cfg.AxesScaling='Auto';
    if isprop(cfg,'Title'), cfg.Title=strrep(titles{k},'_',' '); end
end
end
function route_top(model)
% Replace top-level long wires with paired named local tags.
lines=find_system(model,'FindAll','on','SearchDepth',1,'Type','line');
sources=[];
for k=1:numel(lines)
    if get_param(lines(k),'LineParent')~=-1, continue; end
    src=get_param(lines(k),'SrcPortHandle'); dst=get_param(lines(k),'DstPortHandle');
    if src~=-1 && all(dst~=-1), sources(end+1)=src; end %#ok<AGROW>
end
sources=unique(sources,'stable');
for k=1:numel(sources)
    src=sources(k); line=get_param(src,'Line');
    if line==-1, continue; end
    dst=get_param(line,'DstPortHandle'); delete_line(line);
    sourceblock=get_param(src,'Parent'); sourceName=get_param(sourceblock,'Name');
    port=get_param(src,'PortNumber'); tag=['M2R_' short_tag(sourceName,port)];
    tag=regexprep(tag,'[^a-zA-Z0-9_]','_'); g=['Publish_' tag];
    b(model,'simulink/Signal Routing/Goto',g,'GotoTag',tag,'TagVisibility','global','ShowName','off');
    gp=get_param([model '/' g],'PortHandles'); add_line(model,src,gp.Inport,'autorouting','on');
    for j=1:numel(dst)
        f=sprintf('Read_%s_%d',tag,j); b(model,'simulink/Signal Routing/From',f,'GotoTag',tag,'ShowName','off');
        fp=get_param([model '/' f],'PortHandles'); add_line(model,fp.Outport,dst(j),'autorouting','on');
    end
end
end
function pack_support(model)
% Put logging taps and firmware state memory inside their own clear modules.
mux=[model '/Simulation_log_34']; ports=get_param(mux,'PortHandles'); handles=get_param(mux,'Handle');
for k=1:numel(ports.Inport)
    line=get_param(ports.Inport(k),'Line'); src=get_param(line,'SrcBlockHandle'); handles(end+1)=src;
end
Simulink.BlockDiagram.createSubsystem(handles,'Name','Logging_taps');
ports=get_param([model '/Logging_taps'],'PortHandles'); line=get_param(ports.Outport(1),'Line');
dest=get_param(line,'DstBlockHandle');
% Output runs to a Goto; results input is a From with the matching tag.
tag=get_param(dest,'GotoTag'); allFrom=find_system(model,'SearchDepth',1,'BlockType','From','GotoTag',tag);
handles=[get_param([model '/Logging_taps'],'Handle'),dest,get_param([model '/09_Results_11_Scopes'],'Handle')];
for k=1:numel(allFrom), handles(end+1)=get_param(allFrom{k},'Handle'); end
Simulink.BlockDiagram.createSubsystem(handles,'Name','09_Results_and_Logging');

items={'Previous_command_pu','Previous_Clarke_ab','Previous_Park_dq'}; handles=[];
for k=1:numel(items)
    block=[model '/' items{k}]; ph=get_param(block,'PortHandles');
    handles=[handles,get_param(block,'Handle'),get_param(get_param(ph.Inport(1),'Line'),'SrcBlockHandle'), ...
        get_param(get_param(ph.Outport(1),'Line'),'DstBlockHandle')]; %#ok<AGROW>
end
Simulink.BlockDiagram.createSubsystem(handles,'Name','10_Firmware_Cycle_Memory');
block=[model '/Observer_split']; ph=get_param(block,'PortHandles');
handles=[get_param(block,'Handle'),get_param(get_param(ph.Inport(1),'Line'),'SrcBlockHandle')];
for k=1:numel(ph.Outport)
    line=get_param(ph.Outport(k),'Line');
    if line~=-1, handles=[handles,get_param(line,'DstBlockHandle')]; end %#ok<AGROW>
end
Simulink.BlockDiagram.createSubsystem(handles,'Name','11_Observer_Output_Decode');
for name={'09_Results_and_Logging','10_Firmware_Cycle_Memory','11_Observer_Output_Decode'}
    Simulink.BlockDiagram.arrangeSystem([model '/' name{1}]);
end
end
function position_tags(model)
% Keep every tag adjacent to its own subsystem port; no long top-level wires.
gotos=find_system(model,'SearchDepth',1,'BlockType','Goto');
for k=1:numel(gotos)
    ph=get_param(gotos{k},'PortHandles'); line=get_param(ph.Inport(1),'Line'); src=get_param(line,'SrcPortHandle');
    xy=get_param(src,'Position'); set_param(gotos{k},'Position',[xy(1)+25 xy(2)-10 xy(1)+230 xy(2)+10]);
end
froms=find_system(model,'SearchDepth',1,'BlockType','From');
for k=1:numel(froms)
    ph=get_param(froms{k},'PortHandles'); line=get_param(ph.Outport(1),'Line'); dest=get_param(line,'DstPortHandle');
    xy=get_param(dest(1),'Position'); set_param(froms{k},'Position',[xy(1)-240 xy(2)-10 xy(1)-25 xy(2)+10]);
end
lines=find_system(model,'FindAll','on','SearchDepth',1,'Type','line');
for k=1:numel(lines), try set_param(lines(k),'Points',[]); catch, end, end
try Simulink.BlockDiagram.routeLine(lines); catch, end
end
function icon(s,caption)
mask=Simulink.Mask.get(s); if isempty(mask), mask=Simulink.Mask.create(s); end
mask.Display=sprintf('disp(''%s'');',caption);
set_param(s,'MaskIconOpaque','opaque','FontSize','14');
end
function group(s,name,items,caption)
handles=zeros(1,numel(items));
for k=1:numel(items), handles(k)=get_param([s '/' items{k}],'Handle'); end
Simulink.BlockDiagram.createSubsystem(handles,'Name',name);
icon([s '/' name],caption);
% Automatic grouping exposes unused auxiliary PI signals as extra ports.
% Keep the visible interface limited to signals actually used by the parent.
path=[s '/' name]; ph=get_param(path,'PortHandles');
for k=numel(ph.Outport):-1:1
    if get_param(ph.Outport(k),'Line')==-1
        outs=find_system(path,'SearchDepth',1,'BlockType','Outport');
        for j=1:numel(outs)
            if str2double(get_param(outs{j},'Port'))==k, delete_block(outs{j}); break; end
        end
    end
end
end
function tidy_control_views(model)
s=[model '/05_Speed_Ramp_and_PI'];
names={'target_rpm','estimated_rpm','01_Observer_Warmup_and_Command_Shaping','02_Speed_Slew_Limiter','03_Speed_PI_Output_and_Polarity','Iq_reference','ramped_rpm'};
pos={[30 220 60 240],[720 30 750 50],[150 170 430 310],[530 170 800 310],[900 130 1250 340],[1390 170 1420 190],[1390 380 1420 400]};
for k=1:numel(names), set_param([s '/' names{k}],'Position',pos{k},'FontSize','12'); end
route_local(s);
s=[model '/06_Id_Iq_PI_and_Voltage_Circle'];
names={'Idq_measured','Split_Id_Iq','01_Id_PI_and_Filter','02_Voltage_Circle','03_Iq_PI_and_Filter','Iq_reference','Vd_Vq','Vdq_pu'};
pos={[30 240 60 260],[150 170 155 330],[280 70 600 230],[730 250 1050 410],[1150 280 1490 470],[950 520 980 540],[1630 140 1635 450],[1750 290 1780 310]};
for k=1:numel(names), set_param([s '/' names{k}],'Position',pos{k},'FontSize','12'); end
route_local(s);
end
function route_local(s)
lines=find_system(s,'FindAll','on','SearchDepth',1,'Type','line');
try Simulink.BlockDiagram.routeLine(lines); catch, end
end
function tag=short_tag(name,port)
switch name
    case '01_Mode2_Command', tags={'Target_rpm','Mode_2'};
    case '02_PWM_Voltage_and_Current_Sampling', tags={'Observer_frame10'};
    case '03_Hybrid_Flux_Observer', tags={'Observer_y11'};
    case '04_Clarke_Park', tags={'Measured_Idq_A','Measured_ab_A','Park_cos_sin'};
    case '05_Speed_Ramp_and_PI', tags={'Iq_reference_A','Ramped_rpm'};
    case '06_Id_Iq_PI_and_Voltage_Circle', tags={'Vdq_command_pu'};
    case '07_InversePark_SVPWM_Inverter', tags={'Command_ab_pu','Bridge_voltage_V','CCR_abc'};
    case '08_PMSM_Electrical_Mechanical_Plant', tags={'Current_abc_A','Physical_ab_A','Actual_rpm','Actual_theta_rad','Torque_Nm'};
    case 'Previous_command_pu', tags={'Previous_command_pu'};
    case 'Previous_Clarke_ab', tags={'Previous_Clarke_ab'};
    case 'Previous_Park_dq', tags={'Previous_Park_dq'};
    case 'Observer_split', tags={'Estimated_theta_rad','Estimated_Hz','Estimated_rpm'};
    case 'Simulation_log_34', tags={'Full_log34'};
    otherwise, tags={name};
end
tag=tags{port};
end


