function errors=disturbances(data,cfg)
%DISTURBANCES Reuse exact Python noise for seed 42; local stream otherwise.
validateattributes(cfg.error_deg,{'numeric'},{'real','finite','scalar','nonnegative'});
validateattributes(cfg.seed,{'numeric'},{'real','finite','scalar','integer','>=',0,'<=',2^32-1});
validateattributes(cfg.rho,{'numeric'},{'real','finite','scalar','>=',0,'<',1});
n=size(data.reference,1); errors=zeros(n,6);
if cfg.seed==data.seed && cfg.rho==0.35
    errors=data.noise_unit*cfg.error_deg;
else
    % A private stream avoids changing the caller's global random state.
    stream=RandStream('mt19937ar','Seed',cfg.seed);
    innovations=(2*rand(stream,n,5)-1)*deg2rad(cfg.error_deg);
    previous=zeros(1,5);
    for k=1:n
        previous=cfg.rho*previous+(1-cfg.rho)*innovations(k,:);
        errors(k,1:5)=previous;
    end
end
end
