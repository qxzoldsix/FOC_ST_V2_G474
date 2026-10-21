function start_flux_observer(action,startup_mode)
% Single entry point: 'open' (default), 'run', 'build', or 'verify'.
if nargin==0, action = 'open'; end
if nargin<2, startup_mode='aligned'; end
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root,'scripts'),fullfile(root,'scripts','verification'),fullfile(root,'model'));
switch lower(action)
    case 'open'
        model_file = fullfile(root,'model','G474_Hybrid_Flux_Observer.slx');
        if ~isfile(model_file), build_flux_observer([],startup_mode); end
        load_system(fullfile(root,'model','G474_Firmware_PI.slx'));
        open_system(model_file);
        set_param('G474_Hybrid_Flux_Observer','ZoomFactor','FitSystem');
    case 'run'
        run_flux_observer_demo(true,startup_mode);
    case 'build'
        build_flux_observer([],startup_mode);
    case 'verify'
        validate_flux_observer;
        build_flux_observer([],startup_mode);
    otherwise
        error('Use open, run, build, or verify.');
end
end
