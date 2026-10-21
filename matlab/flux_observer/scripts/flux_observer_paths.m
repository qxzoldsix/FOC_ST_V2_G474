function d = flux_observer_paths()
% All generated files have one explicit destination under this artifact.
d.root = fileparts(fileparts(mfilename('fullpath')));
d.scripts = fullfile(d.root,'scripts');
d.model = fullfile(d.root,'model');
d.results = fullfile(d.root,'results');
d.work = fullfile(d.root,'work');
d.verification = fullfile(d.scripts,'verification');
d.reference = fullfile(d.verification,'reference');
for folder = {d.model,d.results,d.work}
    if ~isfolder(folder{1}), mkdir(folder{1}); end
end
addpath(d.scripts,d.verification,d.model);
end
