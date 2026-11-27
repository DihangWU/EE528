function tests=test_controllers
% Behavioral tests for dynamics, limits, causality, replay and Python parity.
tests=functiontests(localfunctions);
end
function setupOnce(testCase)
root=fileparts(fileparts(mfilename('fullpath'))); addpath(root); setup();
testCase.TestData.data=nexarm.trajectory.loadReference();
testCase.TestData.golden=load(fullfile(root,'data','python_baseline.mat'));
end
function testMotorStepAndZeroTau(testCase)
motor=nexarm.model.MotorModel([-1 -1],[1 1],[.2 0],1/30,[0 0]);
actual=motor.step([.5 .5],[0 .01]);
verifyEqual(testCase,actual,[(1-exp(-1/6))*.5 .51],'AbsTol',1e-14);
for k=1:200, actual=motor.step([.5 .5],[0 0]); end
verifyEqual(testCase,actual,[.5 .5],'AbsTol',1e-12);
motor.reset([0 0]); verifyEqual(testCase,motor.nominal,[0 0]);
end
function testLqrClosedLoopStability(testCase)
c=LQRController(-ones(1,6),ones(1,6),ones(1,6)*3.14,[.6 .5 .6 .4 0 0]);
verifyTrue(testCase,all(abs(c.a-c.b.*c.gain)<1));
verifyEqual(testCase,c.gain(5:6),[0 0],'AbsTol',1e-14);
end
function testActiveConstraints(testCase)
c=MPCController(-1,1,.15,0,struct('dt',.1,'horizon',6,'control_horizon',4,'solver','dual'));
[u,info]=c.command(0,0,.8*ones(6,1));
verifyEqual(testCase,c.active_qps,1); verifyEqual(testCase,c.fallbacks,0);
verifyTrue(testCase,all(info.success)); verifyGreaterThan(testCase,u,0);
verifyLessThanOrEqual(testCase,u,.015+1e-8);
verifyLessThanOrEqual(testCase,max(abs(info.increments),[],'all'),.015+1e-8);
verifyLessThanOrEqual(testCase,max(abs(info.commands),[],'all'),1+1e-8);
verifyLessThanOrEqual(testCase,max(abs(info.predictions),[],'all'),1+1e-8);
verifyLessThanOrEqual(testCase,max(abs(diff([0;info.predictions])),[],'all'),.015+1e-8);
end
function testInfeasibleFallback(testCase)
c=MPCController(-1,1,.1,0,struct('dt',.1,'horizon',4,'control_horizon',3,'solver','dual'));
[u,info]=c.command(1,.9,zeros(4,1));
verifyEqual(testCase,c.fallbacks,1); verifyFalse(testCase,info.success);
verifyGreaterThanOrEqual(testCase,u,.89-1e-8); verifyLessThanOrEqual(testCase,u,.91+1e-8);
end
function testKnownQpOptimum(testCase)
% Independent analytic solution: x=1/3, y=2/3, active x+y<=1.
[z,success,info]=nexarm.internal.solveQP(diag([2 4]),[-2;-4],...
    [1 1;-1 0;0 -1],[1;0;0],'dual');
verifyTrue(testCase,success); verifyEqual(testCase,z,[1/3;2/3],'AbsTol',1e-9);
verifyLessThanOrEqual(testCase,info.primal_residual,1e-10);
end
function testJointPositionAndCommandRate(testCase)
types={@() PIDController(-1,1,.15), @() LQRController(-1,1,.15,.2),...
    @() MPCController(-1,1,.15,.2,struct('horizon',6,'control_horizon',4,'solver','dual'))};
for j=1:numel(types)
    c=types{j}(); previous=.95; measured=.95;
    for k=1:15
        u=c.command(measured,previous,2*ones(c.horizon,1));
        verifyLessThanOrEqual(testCase,abs(u-previous),.15/30+1e-8);
        verifyLessThanOrEqual(testCase,abs(u),1+1e-8);
        previous=u; measured=u;
    end
end
end
function testPidAntiWindup(testCase)
c=PIDController(-1,1,.1); c.command(0,0,1);
verifyEqual(testCase,c.integral,0); verifyEqual(testCase,c.steps,1);
c.reset(); verifyEqual(testCase,c.steps,0); verifyEmpty(testCase,c.previous_error);
end
function testResetAndCausality(testCase)
for mode={'pid','lqr','mpc'}
    cfg=default_config(); cfg.mode=mode{1}; cfg.solver='dual';
    data=shortData(testCase.TestData.data,130);
    errors=nexarm.simulation.disturbances(data,cfg);
    first=nexarm.simulation.run(data,cfg,errors);
    second=nexarm.simulation.run(data,cfg,errors);
    verifyEqual(testCase,first.commands,second.commands,'AbsTol',0);
    errors(101:end,:)=-errors(101:end,:);
    future=nexarm.simulation.run(data,cfg,errors);
    verifyEqual(testCase,first.commands(1:100,:),future.commands(1:100,:),'AbsTol',0);
    % Explicit reset of the same controller handle clears the observer/history.
    c=makeController(mode{1},data,cfg); q=data.reference(1,:);
    preview=data.reference(1:c.horizon,:); u=c.command(q,q,preview);
    c.command(q,u,preview); c.reset();
    verifyEqual(testCase,c.steps,0); verifyEqual(testCase,c.command(q,q,preview),u,'AbsTol',0);
end
end
function testNoiseAndZeroError(testCase)
data=testCase.TestData.data; cfg=default_config();
errors=nexarm.simulation.disturbances(data,cfg);
verifyLessThanOrEqual(testCase,max(abs(errors),[],'all'),deg2rad(cfg.error_deg));
verifyEqual(testCase,errors(:,6),zeros(size(errors,1),1));
before=rng; cfg.seed=7; nexarm.simulation.disturbances(data,cfg); verifyEqual(testCase,rng,before);
cfg.mode='off'; cfg.motor_tau=zeros(1,6); cfg.error_deg=0;
result=nexarm.simulation.run(data,cfg);
verifyEqual(testCase,result.actual,data.reference,'AbsTol',0);
verifyLessThan(testCase,result.metrics.pen_xyz_rms_mm,1e-9);
end
function testPythonFullTrajectoryParity(testCase)
data=testCase.TestData.data; golden=testCase.TestData.golden;
for mode={'off','pid','lqr','mpc'}
    cfg=default_config(); cfg.mode=mode{1}; cfg.solver='dual';
    result=nexarm.simulation.run(data,cfg); expected=golden.(cfg.mode);
    if strcmp(cfg.mode,'mpc')
        % Legacy SLSQP stops on objective change (ftol=1e-10). Its control
        % increments can differ although the physical track agrees closely.
        verifyEqual(testCase,result.actual,expected.actual,'AbsTol',1e-4);
        verifyEqual(testCase,result.commands,expected.commands,'AbsTol',1e-3);
        verifyEqual(testCase,result.tips,expected.tips,'AbsTol',2e-5);
        % Check a separate, tighter solve of the identical Python QP too.
        tight=golden.mpc_tight;
        verifyEqual(testCase,result.actual,tight.actual,'AbsTol',2e-6);
        verifyEqual(testCase,result.commands,tight.commands,'AbsTol',2e-5);
        verifyEqual(testCase,result.tips,tight.tips,'AbsTol',1e-6);
    else
        verifyEqual(testCase,result.actual,expected.actual,'AbsTol',1e-9);
        verifyEqual(testCase,result.commands,expected.commands,'AbsTol',1e-9);
        verifyEqual(testCase,result.tips,expected.tips,'AbsTol',1e-9);
    end
    verifyEqual(testCase,result.statistics.fallbacks,double(expected.fallbacks));
end
end
function testMetricStrokeDistance(testCase)
data=struct('ink',[false;true;true;false;true],...
    'reference_tips',[0 0 0;1 0 0;2 0 0;0 10 0;1 10 0],...
    'reference',zeros(5,6));
tips=data.reference_tips; tips(:,2)=tips(:,2)+.1;
m=nexarm.metrics.evaluate(zeros(5,6),tips,data);
verifyEqual(testCase,m.pen_xyz_rms_mm,100,'AbsTol',1e-10);
verifyEqual(testCase,m.pen_xy_contour_rms_mm,100,'AbsTol',1e-10);
line=nexarm.visualization.strokeLine(tips,data.ink);
verifyEqual(testCase,sum(isnan(line(:,1))),2);
end
function testInvalidInputs(testCase)
verifyError(testCase,@() MPCController(-1,1,3,0,struct('horizon',4,'control_horizon',5)),...
    'MATLAB:notLessEqual');
verifyError(testCase,@() nexarm.internal.mergeConfig(struct('motor_tua',.2)),'nexarm:UnknownSetting');
verifyError(testCase,@() nexarm.model.motorCoefficients([.1 .2],6,1/30),'nexarm:InvalidTau');
end
function data=shortData(data,n)
data.reference=data.reference(1:n,:); data.reference_tips=data.reference_tips(1:n,:);
data.ink=data.ink(1:n); data.time=data.time(1:n); data.noise_unit=data.noise_unit(1:n,:);
end
function c=makeController(mode,data,cfg)
m=data.model;
switch mode
    case 'pid', c=PIDController(m.lower,m.upper,m.velocity);
    case 'lqr', c=LQRController(m.lower,m.upper,m.velocity,cfg.motor_tau);
    case 'mpc', c=MPCController(m.lower,m.upper,m.velocity,cfg.motor_tau,cfg);
end
end
