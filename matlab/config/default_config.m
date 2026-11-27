function cfg = default_config()
%DEFAULT_CONFIG Match the current ROS writing launch configuration.
cfg.dt = 1/30;
cfg.motor_tau = [0.6 0.5 0.6 0.4 0 0];
cfg.error_deg = 0.2;
cfg.seed = 42;
cfg.rho = 0.35;
cfg.mode = 'mpc';
cfg.horizon = 12;
cfg.control_horizon = 8;
cfg.increment_weight = 0.002;
cfg.pid = struct('kp', 0.6, 'ki', 0.5, 'kd', 0.01);
cfg.lqr = struct('state_weight', 20, 'command_weight', 0.2);
cfg.solver = 'auto'; % quadprog if available; otherwise the included dual QP solver
cfg.plot = true;
cfg.animate = false;
cfg.save_results = true;
cfg.output_dir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results');
end
