classdef LQRController < handle
    %LQRCONTROLLER Discrete scalar DARE per motor; bounded issued commands.
    properties (SetAccess = private)
        lower; upper; delta; a; b; gain
        horizon = 1
        steps = 0; fallbacks = 0; active_qps = 0; solve_times = []
    end
    methods
        function obj = LQRController(lower,upper,velocity,tau,dt,Q,R)
            if nargin<4, tau=0.2; end
            if nargin<5, dt=1/30; end
            if nargin<6, Q=20; end
            if nargin<7, R=0.2; end
            [obj.lower,obj.upper,obj.delta]=nexarm.internal.jointModel(lower,upper,velocity,dt);
            validateattributes(Q,{'numeric'},{'real','finite','positive','scalar'});
            validateattributes(R,{'numeric'},{'real','finite','positive','scalar'});
            [~,obj.a]=nexarm.model.motorCoefficients(tau,numel(obj.lower),dt); obj.b=1-obj.a;
            % Positive root of b^2 P^2 + [R(1-a^2)-Q b^2]P - QR = 0.
            % This equals solve_discrete_are without requiring a toolbox.
            c=R*(1-obj.a.^2)-Q*obj.b.^2;
            disc=sqrt(c.^2+4*obj.b.^2*Q*R); p=zeros(size(c));
            positive=c>=0;
            p(positive)=2*Q*R./(c(positive)+disc(positive));
            p(~positive)=(-c(~positive)+disc(~positive))./(2*obj.b(~positive).^2);
            obj.gain=obj.b.*p.*obj.a./(R+obj.b.^2.*p);
            obj.reset();
        end
        function reset(obj)
            obj.steps=0; obj.fallbacks=0; obj.active_qps=0; obj.solve_times=[];
        end
        function [command,info]=command(obj,measured,previous,reference)
            started=tic;
            [measured,previous,reference]=nexarm.internal.feedback(measured,previous,reference,numel(obj.lower),1);
            raw=reference(1,:)-obj.gain.*(measured-reference(1,:));
            command=min(min(obj.upper,previous+obj.delta),max(max(obj.lower,previous-obj.delta),raw));
            obj.steps=obj.steps+1; obj.solve_times(end+1)=toc(started);
            info=struct('saturated',raw~=command);
        end
    end
end
