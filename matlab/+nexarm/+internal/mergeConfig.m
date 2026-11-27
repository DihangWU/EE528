function cfg=mergeConfig(overrides)
%MERGECONFIG Merge explicit settings into defaults; reject misspelled keys.
cfg=default_config();
if nargin<1 || isempty(overrides), return; end
assert(isstruct(overrides) && isscalar(overrides),'nexarm:InvalidConfig','Configuration must be a scalar struct.');
keys=fieldnames(overrides);
for k=1:numel(keys)
    key=keys{k};
    assert(isfield(cfg,key),'nexarm:UnknownSetting','Unknown configuration field: %s',key);
    if isstruct(cfg.(key))
        names=fieldnames(overrides.(key));
        for j=1:numel(names)
            assert(isfield(cfg.(key),names{j}),'nexarm:UnknownSetting','Unknown %s setting: %s',key,names{j});
            cfg.(key).(names{j})=overrides.(key).(names{j});
        end
    else
        cfg.(key)=overrides.(key);
    end
end
end
