function library = build_firmware_pi_library()
% Native discrete equations, copied from flux.c and PI_Cale.c (c43e794).
d = flux_observer_paths(); p = flux_observer_params(); library = 'G474_Firmware_PI';
if bdIsLoaded(library), close_system(library,0); end
new_system(library,'Library');
make_pi(library,'Flux_Compensation_PI','flux',num2str(p.Comp_Kp,17),num2str(p.Comp_Ki,17));
make_pi(library,'PLL_PI','pll',num2str(p.PLL_kp,17),num2str(p.PLL_ki,17));
make_pi(library,'FOC_Speed_PI','foc','0.01','0.0001');
make_pi(library,'FOC_Id_PI','foc','0.01','0.00001');
make_pi(library,'FOC_Iq_PI','foc','0.01','0.00001');
names = {'Flux_Compensation_PI','PLL_PI','FOC_Speed_PI','FOC_Id_PI','FOC_Iq_PI'};
positions = {[60 100 270 190],[370 100 580 190], ...
    [60 290 270 400],[370 290 580 400],[680 290 890 400]};
for k=1:numel(names), set_param([library '/' names{k}],'Position',positions{k}); end
a = Simulink.Annotation(library,sprintf(['c43e794: original firmware PI equations\n' ...
    'Top: flux.c compensation / PLL. Bottom: PI_Cale.c PID_controller.\n' ...
    'K_i is a discrete coefficient; no extra Ts multiplier.\n' ...
    'FOC anti-windup uses PREVIOUS unclamped v1, including equality.']));
a.Position = [60 5 890 80];
save_system(library,fullfile(d.model,[library '.slx']));
set_param(library,'Lock','on');
end

function make_pi(lib,name,kind,kp,ki)
s = [lib '/' name]; add_block('built-in/Subsystem',s);
mask = Simulink.Mask.create(s);
mask.addParameter('Type','edit','Name','Kp','Prompt','Kp (source coefficient)','Value',kp);
mask.addParameter('Type','edit','Name','Ki','Prompt','Ki (per call, already discrete)','Value',ki);
mask.addParameter('Type','edit','Name','Ts','Prompt','Execution period (s)','Value','5e-5');
if strcmp(kind,'flux')
    inputs = {'error','integral_enable'}; outputs = {'Ucomp','integral'};
    note = ['flux.c: Comp_Interg += error*Comp_Ki; Ucomp = error*Comp_Kp + Comp_Interg.' ...
        newline 'No integral/output saturation in this firmware branch. SMO selection freezes PI memory.'];
elseif strcmp(kind,'pll')
    inputs = {'PLL_Err','step_limit'}; outputs = {'PLL_Ui','PLL_Interg'};
    note = ['flux.c: I = sat(I_old + Ki*error); PLL_Ui = sat(Kp*error + I).' ...
        newline 'Both clamps use +/- 2*pi*500*Ts. Output unit: rad/sample.'];
else
    inputs = {'Ref','Fbk','Umax','Umin'}; outputs = {'Out','ui','v1','up'};
    note = ['PI_Cale.c PID_controller:' newline ...
        'up = Ref-Fbk; freeze = (v1_old >= Umax || v1_old <= Umin);' newline ...
        'ui = sat(freeze ? i1_old : Ki*up+i1_old); i1 = ui;' newline ...
        'v1 = Kp*up+ui; Out = sat(v1). OutF filtering is outside this function.'];
end
for k=1:numel(inputs), add_block('built-in/Inport',[s '/' inputs{k}],'Port',num2str(k)); end
for k=1:numel(outputs), add_block('built-in/Outport',[s '/' outputs{k}],'Port',num2str(k)); end
if strcmp(kind,'foc')
    b(s,'Sum','Error','Inputs','+-'); w(s,'Ref/1','Error/1'); w(s,'Fbk/1','Error/2');
    w(s,'Error/1','up/1');
    delay(s,'Previous_v1');
    b(s,'RelationalOperator','At_upper_limit','Operator','>=');
    b(s,'RelationalOperator','At_lower_limit','Operator','<=');
    w(s,'Previous_v1/1','At_upper_limit/1'); w(s,'Umax/1','At_upper_limit/2');
    w(s,'Previous_v1/1','At_lower_limit/1'); w(s,'Umin/1','At_lower_limit/2');
    b(s,'Logic','Freeze_integral','Operator','OR','Inputs','2');
    w(s,'At_upper_limit/1','Freeze_integral/1'); w(s,'At_lower_limit/1','Freeze_integral/2');
    error = 'Error/1';
else
    error = [inputs{1} '/1'];
end
b(s,'Gain','Proportional','Gain','Kp'); w(s,error,'Proportional/1');
b(s,'Gain','Integral_increment','Gain','Ki'); w(s,error,'Integral_increment/1');
delay(s,'Previous_integral'); b(s,'Sum','Integral_candidate','Inputs','++');
w(s,'Previous_integral/1','Integral_candidate/1'); w(s,'Integral_increment/1','Integral_candidate/2');
if strcmp(kind,'flux')
    b(s,'Switch','Enable_integral','Criteria','u2 ~= 0');
    w(s,'Integral_candidate/1','Enable_integral/1'); w(s,'integral_enable/1','Enable_integral/2');
    w(s,'Previous_integral/1','Enable_integral/3'); integral = 'Enable_integral/1';
    w(s,integral,'integral/1');
elseif strcmp(kind,'pll')
    b(s,'Gain','Negative_limit','Gain','-1'); w(s,'step_limit/1','Negative_limit/1');
    integral = clamp(s,'Integral_limit','Integral_candidate/1','step_limit/1','Negative_limit/1');
    w(s,integral,'PLL_Interg/1');
else
    b(s,'Switch','Integral_separation','Criteria','u2 ~= 0');
    w(s,'Previous_integral/1','Integral_separation/1'); w(s,'Freeze_integral/1','Integral_separation/2');
    w(s,'Integral_candidate/1','Integral_separation/3');
    integral = clamp(s,'Integral_limit','Integral_separation/1','Umax/1','Umin/1');
    w(s,integral,'ui/1');
end
w(s,integral,'Previous_integral/1');
b(s,'Sum','Pre_output','Inputs','++'); w(s,'Proportional/1','Pre_output/1'); w(s,integral,'Pre_output/2');
if strcmp(kind,'foc')
    w(s,'Pre_output/1','Previous_v1/1'); w(s,'Pre_output/1','v1/1');
    output = clamp(s,'Output_limit','Pre_output/1','Umax/1','Umin/1');
elseif strcmp(kind,'pll')
    output = clamp(s,'Output_limit','Pre_output/1','step_limit/1','Negative_limit/1');
else
    output = 'Pre_output/1';
end
w(s,output,[outputs{1} '/1']);
Simulink.BlockDiagram.arrangeSystem(s);
a = Simulink.Annotation(s,note); a.Position = [15 -110 1000 -15];
display = '';
for k=1:numel(inputs), display = [display sprintf('port_label(''input'',%d,''%s'');',k,inputs{k})]; end %#ok<AGROW>
for k=1:numel(outputs), display = [display sprintf('port_label(''output'',%d,''%s'');',k,outputs{k})]; end %#ok<AGROW>
mask.Display = display;
set_param(s,'BackgroundColor','lightBlue','OpenFcn','open_system(gcb,''force'')');
end

function output = clamp(s,name,input,upper,lower)
b(s,'MinMax',[name '_upper'],'Function','min','Inputs','2');
w(s,input,[name '_upper/1']); w(s,upper,[name '_upper/2']);
b(s,'MinMax',[name '_lower'],'Function','max','Inputs','2');
w(s,[name '_upper/1'],[name '_lower/1']); w(s,lower,[name '_lower/2']);
output = [name '_lower/1'];
end
function delay(s,name)
add_block('simulink/Discrete/Unit Delay',[s '/' name],'SampleTime','Ts','InitialCondition','0');
end
function b(s,kind,name,varargin)
paths = struct('Sum','Math Operations/Sum','Gain','Math Operations/Gain', ...
    'RelationalOperator','Logic and Bit Operations/Relational Operator', ...
    'Logic','Logic and Bit Operations/Logical Operator','Switch','Signal Routing/Switch', ...
    'MinMax','Math Operations/MinMax');
add_block(['simulink/' paths.(kind)],[s '/' name],varargin{:});
end
function w(s,a,b)
add_line(s,a,b,'autorouting','on');
end
