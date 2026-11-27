classdef PIDController < handle
    %PIDCONTROLLER Reference feedforward, filtered derivative, anti-windup.
    properties (SetAccess = private)
        lower; upper; delta; dt; kp; ki; kd
        horizon = 1
        integral; derivative; previous_error
        steps = 0; fallbacks = 0; active_qps = 0; solve_times = []
    end
    methods
        function obj = PIDController(lower, upper, velocity, dt, kp, ki, kd)
            if nargin < 4, dt = 1/30; end
            if nargin < 5, kp = 0.6; end
            if nargin < 6, ki = 0.5; end
            if nargin < 7, kd = 0.01; end
            [obj.lower,obj.upper,obj.delta] = nexarm.internal.jointModel(lower,upper,velocity,dt);
            validateattributes([kp ki kd], {'numeric'}, {'real','finite','nonnegative','size',[1 3]});
            obj.dt=dt; obj.kp=kp; obj.ki=ki; obj.kd=kd; obj.reset();
        end
        function reset(obj)
            obj.integral=zeros(size(obj.lower)); obj.derivative=obj.integral;
            obj.previous_error=[]; obj.steps=0; obj.fallbacks=0;
            obj.active_qps=0; obj.solve_times=[];
        end
        function [command,info] = command(obj,measured,previous,reference)
            started=tic;
            [measured,previous,reference]=nexarm.internal.feedback(measured,previous,reference,numel(obj.lower),1);
            error=reference(1,:)-measured;
            if ~isempty(obj.previous_error)
                obj.derivative=0.8*obj.derivative+0.2*(error-obj.previous_error)/obj.dt;
            end
            integral=obj.integral+error*obj.dt;
            raw=reference(1,:)+obj.kp*error+obj.ki*integral+obj.kd*obj.derivative;
            low=max(obj.lower,previous-obj.delta); high=min(obj.upper,previous+obj.delta);
            command=min(high,max(low,raw));
            accept=(raw-command).*error <= 0;
            obj.integral(accept)=integral(accept); obj.previous_error=error;
            obj.steps=obj.steps+1; obj.solve_times(end+1)=toc(started);
            info=struct('saturated',raw~=command);
        end
    end
end
