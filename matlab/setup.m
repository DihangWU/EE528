function root = setup()
%SETUP Add the MATLAB implementation and its three controller files to path.
root = fileparts(mfilename('fullpath'));
addpath(root, fullfile(root, 'algorithm'), fullfile(root, 'config'));
end
