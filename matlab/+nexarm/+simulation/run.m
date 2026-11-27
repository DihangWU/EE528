function result=run(data,cfg,errors)
%RUN Execute every discrete sample. Future noise is never passed to control.
cfg.mode=validatestring(cfg.mode,{'off','pid','lqr','mpc'});
assert(abs(cfg.dt-data.dt)<1e-12,'nexarm:SampleTimeMismatch',...
    'Reference was sampled at %.12g s; regenerate it to change dt.',data.dt);
if nargin<3, errors=nexarm.simulation.disturbances(data,cfg); end
reference=data.reference; n=size(reference,1); model=data.model;
validateattributes(errors,{'numeric'},{'real','finite','size',size(reference)});
controller=[];
switch cfg.mode
    case 'pid'
        controller=PIDController(model.lower,model.upper,model.velocity,cfg.dt,cfg.pid.kp,cfg.pid.ki,cfg.pid.kd);
    case 'lqr'
        controller=LQRController(model.lower,model.upper,model.velocity,cfg.motor_tau,cfg.dt,cfg.lqr.state_weight,cfg.lqr.command_weight);
    case 'mpc'
        controller=MPCController(model.lower,model.upper,model.velocity,cfg.motor_tau,cfg);
end
motor=nexarm.model.MotorModel(model.lower,model.upper,cfg.motor_tau,cfg.dt,reference(1,:));
previous=reference(1,:); measured=previous;
actual=zeros(size(reference)); commands=actual; tips=zeros(n,3);
for k=1:n
    if isempty(controller)
        command=reference(k,:);
    else
        indices=min(k:k+controller.horizon-1,n);
        command=controller.command(measured,previous,reference(indices,:));
    end
    % Observe this sample's output disturbance only after issuing its command.
    measured=motor.step(command,errors(k,:)); previous=command;
    actual(k,:)=measured; commands(k,:)=command;
    tips(k,:)=nexarm.model.forwardKinematics(model,measured);
end
stats=struct('steps',n,'fallbacks',0,'active_qps',0,'solve_mean_ms',0,'solve_p99_ms',0);
if ~isempty(controller)
    stats.fallbacks=controller.fallbacks; stats.active_qps=controller.active_qps;
    stats.solve_mean_ms=mean(controller.solve_times)*1000;
    ordered=sort(controller.solve_times); stats.solve_p99_ms=ordered(max(1,ceil(0.99*numel(ordered))))*1000;
end
result=struct('mode',cfg.mode,'config',cfg,'time',data.time,'actual',actual,...
    'commands',commands,'tips',tips,'errors',errors,'statistics',stats);
result.metrics=nexarm.metrics.evaluate(actual,tips,data);
end
