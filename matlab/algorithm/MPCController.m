classdef MPCController < handle
    %MPCCONTROLLER Linear constrained incremental MPC, one QP per servo.
    % Uses prior feedback and known reference preview, never future noise.
    properties (SetAccess = private)
        lower; upper; delta; dt; horizon; control_horizon; rho
        motor_tau; a; L; D; M; G; H; matrices; weights; decay; solver
        nominal_state=[]
        steps=0; fallbacks=0; active_qps=0; solve_times=[]
    end
    methods
        function obj=MPCController(lower,upper,velocity,tau,options)
            if nargin<4, tau=0.2; end
            if nargin<5, options=struct(); end
            defaults=struct('dt',1/30,'horizon',12,'control_horizon',8,...
                'increment_weight',0.002,'rho',0.35,'solver','auto');
            keys=fieldnames(defaults);
            for k=1:numel(keys)
                if ~isfield(options,keys{k}), options.(keys{k})=defaults.(keys{k}); end
            end
            [obj.lower,obj.upper,obj.delta]=nexarm.internal.jointModel(lower,upper,velocity,options.dt);
            validateattributes(options.horizon,{'numeric'},{'scalar','integer','>=',1,'<=',100});
            validateattributes(options.control_horizon,{'numeric'},{'scalar','integer','>=',1,'<=',options.horizon});
            validateattributes(options.increment_weight,{'numeric'},{'real','scalar','finite','positive'});
            validateattributes(options.rho,{'numeric'},{'real','scalar','finite','>=',0,'<',1});
            obj.solver=validatestring(options.solver,{'auto','dual','quadprog'});
            obj.dt=options.dt; obj.horizon=options.horizon; obj.control_horizon=options.control_horizon;
            obj.rho=options.rho;
            [obj.motor_tau,obj.a]=nexarm.model.motorCoefficients(tau,numel(obj.lower),obj.dt);
            np=obj.horizon; nc=obj.control_horizon;
            obj.L=tril(ones(np,nc)); obj.D=eye(np)-diag(ones(np-1,1),-1);
            obj.decay=obj.rho.^(1:np).'; obj.weights=ones(np,1); obj.weights(end)=2;
            [rows,cols]=ndgrid(0:np-1,0:nc-1); powers=max(0,rows-cols+1);
            for j=1:numel(obj.lower)
                m=(1-obj.a(j).^powers).*(rows>=cols);
                obj.M{j}=m; obj.G{j}=obj.D*m;
                obj.H{j}=2*(m.'*(obj.weights.*m)+options.increment_weight*eye(nc));
                obj.matrices{j}=[obj.L;m;obj.G{j}];
            end
            obj.reset();
        end
        function reset(obj)
            obj.nominal_state=[]; obj.steps=0; obj.fallbacks=0; obj.active_qps=0; obj.solve_times=[];
        end
        function [command,info]=command(obj,measured,previous,reference)
            started=tic; n=numel(obj.lower); np=obj.horizon; nc=obj.control_horizon;
            [measured,previous,reference]=nexarm.internal.feedback(measured,previous,reference,n,np);
            if isempty(obj.nominal_state)
                obj.nominal_state=previous;
                obj.nominal_state(obj.motor_tau>0)=measured(obj.motor_tau>0);
            end
            disturbance=measured-obj.nominal_state;
            powers=obj.a.^((1:np).');
            free=powers.*obj.nominal_state+(1-powers).*previous+obj.decay.*disturbance;
            free_delta=obj.D*free; free_delta(1,:)=free_delta(1,:)-measured;
            increments=zeros(nc,n); commands=zeros(np,n); predictions=zeros(np,n);
            success=true(1,n); solver_info=cell(1,n);
            for j=1:n
                matrix=obj.matrices{j}; response=obj.M{j}; hessian=obj.H{j};
                gradient=2*response.'*(obj.weights.*(free(:,j)-reference(:,j)));
                low=[repmat(obj.lower(j)-previous(j),np,1); obj.lower(j)-free(:,j); -obj.delta(j)-free_delta(:,j)];
                high=[repmat(obj.upper(j)-previous(j),np,1); obj.upper(j)-free(:,j); obj.delta(j)-free_delta(:,j)];
                z=-(hessian\gradient);
                feasible=@(v) all(abs(v)<=obj.delta(j)+1e-8) && ...
                    all(matrix*v>=low-1e-8) && all(matrix*v<=high+1e-8);
                if ~feasible(z)
                    obj.active_qps=obj.active_qps+1;
                    A=[matrix;-matrix;eye(nc);-eye(nc)];
                    b=[high;-low;repmat(obj.delta(j),2*nc,1)];
                    [z,ok,solver_info{j}]=nexarm.internal.solveQP(hessian,gradient,A,b,obj.solver);
                    if ~ok || ~feasible(z)
                        obj.fallbacks=obj.fallbacks+1; success(j)=false;
                        move=min(obj.delta(j),max(-obj.delta(j),reference(1,j)-disturbance(j)-previous(j)));
                        move=min(obj.upper(j)-previous(j),max(obj.lower(j)-previous(j),move));
                        z=[move;zeros(nc-1,1)];
                    end
                end
                increments(:,j)=z; commands(:,j)=previous(j)+obj.L*z;
                predictions(:,j)=free(:,j)+response*z;
            end
            command=commands(1,:);
            obj.nominal_state=min(obj.upper,max(obj.lower,obj.a.*obj.nominal_state+(1-obj.a).*command));
            obj.steps=obj.steps+1; obj.solve_times(end+1)=toc(started);
            info=struct('increments',increments,'commands',commands,'predictions',predictions,...
                'success',success,'solver_info',{solver_info});
        end
    end
end
