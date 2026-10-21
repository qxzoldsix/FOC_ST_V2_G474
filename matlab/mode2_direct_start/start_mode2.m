function result = start_mode2(action)
% Standalone MODE=2 experiment; the existing flux_observer project is untouched.
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
    case 'open'
        if ~isfile(fullfile(d.model,'G474_Mode2_Direct_Start.slx')), build_mode2(); end
        open_system(fullfile(d.model,'G474_Mode2_Direct_Start.slx')); result=[];
    otherwise
        error('Use open, build, run, or verify.');
end
end
