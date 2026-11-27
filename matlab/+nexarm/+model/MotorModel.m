classdef MotorModel < handle
    %MOTORMODEL First-order nominal state plus output arrival disturbance.
    properties (SetAccess=private)
        lower; upper; a; nominal; measured
    end
    methods
        function obj=MotorModel(lower,upper,tau,dt,initial)
            [obj.lower,obj.upper,~]=nexarm.internal.jointModel(lower,upper,ones(size(lower)),dt);
            [~,obj.a]=nexarm.model.motorCoefficients(tau,numel(obj.lower),dt);
            obj.reset(initial);
        end
        function reset(obj,initial)
            validateattributes(initial,{'numeric'},{'real','finite','vector','numel',numel(obj.lower)});
            initial=initial(:).';
            assert(all(initial>=obj.lower & initial<=obj.upper),'nexarm:InvalidInitial','Initial state violates limits.');
            obj.nominal=initial; obj.measured=initial;
        end
        function measured=step(obj,command,disturbance)
            validateattributes(command,{'numeric'},{'real','finite','vector','numel',numel(obj.lower)});
            validateattributes(disturbance,{'numeric'},{'real','finite','vector','numel',numel(obj.lower)});
            obj.nominal=min(obj.upper,max(obj.lower,obj.a.*obj.nominal+(1-obj.a).*command(:).'));
            obj.measured=min(obj.upper,max(obj.lower,obj.nominal+disturbance(:).'));
            measured=obj.measured;
        end
    end
end
