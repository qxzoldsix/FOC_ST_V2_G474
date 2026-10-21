function d=mode2_paths()
d.root=fileparts(fileparts(mfilename('fullpath')));
for name={'model','results','work'}
    d.(name{1})=fullfile(d.root,name{1});
    if ~isfolder(d.(name{1})), mkdir(d.(name{1})); end
end
addpath(fullfile(d.root,'scripts'));
end
