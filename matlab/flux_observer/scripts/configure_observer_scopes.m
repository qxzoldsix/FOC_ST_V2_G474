function configure_observer_scopes(model)
% Exactly eleven scopes: original output y(k) -> scope k, one scalar per scope.
s = [model '/03_Results'];
assert(strcmp(get_param(model,'SimulationStatus'),'stopped'),'Stop simulation before changing scopes.');
% Replacing an Inport can detach the parent-level line; preserve its source.
upstream = [];
old_ports = get_param(s,'PortHandles');
if ~isempty(old_ports.Inport)
    old_line = get_param(old_ports.Inport(1),'Line');
    if old_line~=-1, upstream = get_param(old_line,'SrcPortHandle'); end
end
if isempty(upstream) || upstream==-1
    observer_ports = get_param([model '/02_Flux_Observer'],'PortHandles');
    upstream = observer_ports.Outport(1);
end
Simulink.SubSystem.deleteContents(s);
add_block('built-in/Inport',[s '/Observer_outputs_y11'],'Position',[20 385 50 405]);
new_ports = get_param(s,'PortHandles');
if get_param(new_ports.Inport(1),'Line')==-1
    add_line(model,upstream,new_ports.Inport(1),'autorouting','on');
end
add_block('simulink/Signal Routing/Demux',[s '/Split_11_signals'], ...
    'Outputs','11','Position',[95 90 100 755]);
add_line(s,'Observer_outputs_y11/1','Split_11_signals/1','autorouting','on');
names = {'Theta_est_rad','Electrical_frequency_Hz','Mechanical_speed_rpm', ...
    'Active_flux_alpha_Wb','Active_flux_beta_Wb','Stator_flux_alpha_Wb', ...
    'Stator_flux_beta_Wb','PLL_error_Wb','PLL_angle_step_rad_per_sample', ...
    'Corrected_voltage_alpha_V','Corrected_voltage_beta_V'};
titles = {'01_估算电角度_rad','02_估算电频率_Hz','03_机械转速_rpm', ...
    '04_有效磁链alpha_Wb','05_有效磁链beta_Wb','06_定子磁链alpha_Wb', ...
    '07_定子磁链beta_Wb','08_PLL误差_Wb','09_PLL角度增量_rad_per_sample', ...
    '10_补偿电压alpha_V','11_补偿电压beta_V'};
units = {'rad','Hz','rpm','Wb','Wb','Wb','Wb','Wb','rad/sample','V','V'};
for k=1:11
    tag = sprintf('observer_y%02d',k);
    publisher = sprintf('y%02d_%s',k,names{k});
    y = 90+(k-1)*62;
    add_block('simulink/Signal Routing/Goto',[s '/' publisher], ...
        'GotoTag',tag,'TagVisibility','local','Position',[180 y 350 y+26]);
    h = add_line(s,sprintf('Split_11_signals/%d',k),[publisher '/1'],'autorouting','on');
    set_param(h,'Name',names{k});

    x = 560+mod(k-1,3)*500; y = 125+floor((k-1)/3)*235;
    source = sprintf('View%02d_%s',k,names{k});
    add_block('simulink/Signal Routing/From',[s '/' source], ...
        'GotoTag',tag,'Position',[x y+20 x+100 y+46],'ShowName','off');
    scope = [s '/' titles{k}];
    add_block('simulink/Sinks/Scope',scope,'NumInputPorts','1', ...
        'Position',[x+320 y x+395 y+65]);
    h = add_line(s,[source '/1'],[titles{k} '/1'],'autorouting','on');
    set_param(h,'Name',names{k});
    config = get_param(scope,'ScopeConfiguration');
    if isprop(config,'ShowLegend'), config.ShowLegend = true; end
    if isprop(config,'Title'), config.Title = strrep(titles{k},'_',' '); end
    if isprop(config,'YLabel'), config.YLabel = units{k}; end
    if isprop(config,'TimeSpan'), config.TimeSpan = '2'; end
    if isprop(config,'AxesScaling'), config.AxesScaling = 'Auto'; end
    if isprop(config,'OpenAtSimulationStart'), config.OpenAtSimulationStart = false; end
end
add_block('simulink/Sinks/To Workspace',[s '/Complete_log_y01_to_y11'], ...
    'VariableName','observer_y','SaveFormat','Timeseries', ...
    'MaxDataPoints','inf','Decimation','1','Position',[180 810 350 850]);
add_line(s,'Observer_outputs_y11/1','Complete_log_y01_to_y11/1','autorouting','on');
a = Simulink.Annotation(s,sprintf(['11 个独立示波器：原输出 y01~y11 与示波器 01~11 一一对应。\n' ...
    '每个窗口仅显示一条曲线，名称和单位见窗口标题及图例；横轴为时间 (s)。\n' ...
    '仿真后双击需要查看的示波器；完整数据仍记录到 observer_y。']));
a.Position = [540 5 1750 100];
set_param(s,'ZoomFactor','FitSystem');
end
