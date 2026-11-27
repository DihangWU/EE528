function data=loadReference(filename)
%LOADREFERENCE Shared IK trajectory and seed-42 noise, with no ROS dependency.
if nargin<1
    root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
    filename=fullfile(root,'data','reference.mat');
end
assert(isfile(filename),'nexarm:MissingReference',...
    'Missing reference data. Run python3 matlab/tools/export_reference.py from the repository root.');
data=load(filename);
data.time=data.time(:); data.ink=logical(data.ink(:));
data.model.lower=data.model.lower(:).'; data.model.upper=data.model.upper(:).';
data.model.velocity=data.model.velocity(:).'; data.model.revolute=logical(data.model.revolute(:).');
data.model.world_offset=data.model.world_offset(:).';
n=size(data.reference,1);
validateattributes(data.reference,{'numeric'},{'real','finite','size',[n 6]});
assert(numel(data.ink)==n && ~data.ink(1) && any(data.ink),'nexarm:InvalidInk','Invalid pen-down mask.');
assert(all(data.reference>=data.model.lower & data.reference<=data.model.upper,'all'),...
    'nexarm:InvalidReference','Reference violates joint limits.');
end
