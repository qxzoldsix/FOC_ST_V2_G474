function result=start_mode2_repair(action)
% Entry point with a unique name; original model projects remain separate.
if nargin<1, action='open'; end
root=fileparts(mfilename('fullpath')); addpath(fullfile(root,'scripts'));
d=mode2_paths();
switch lower(action)
    case 'build'
        result=build_mode2();
    case 'run'
        build_mode2(); result=run_mode2();
    case 'verify'
        build_mode2(); result=verify_mode2();
    case {'open','reload'}
        model=fullfile(d.model,'G474_Mode2_Direct_Start_Repair.slx');
        if ~isfile(model), build_mode2(); end
        if strcmpi(action,'reload') && bdIsLoaded('G474_Mode2_Direct_Start_Repair')
            set_param('G474_Mode2_Direct_Start_Repair','SimulationCommand','stop');
            close_system('G474_Mode2_Direct_Start_Repair',0);
        end
        open_system(model); result=[];
    otherwise
        error('Use open, reload, build, run, or verify.');
end
end
