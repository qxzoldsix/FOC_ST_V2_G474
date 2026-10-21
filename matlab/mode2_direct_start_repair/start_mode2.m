function result=start_mode2(action)
% Compatibility entry point; prefer the uniquely named start_mode2_repair.
if nargin<1, action='open'; end
result=start_mode2_repair(action);
end
