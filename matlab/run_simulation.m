function result=run_simulation(mode,overrides)
%RUN_SIMULATION Run one mode, plot and optionally animate the writing.
% result=run_simulation('mpc',struct('error_deg',0,'animate',true));
root=fileparts(mfilename('fullpath')); addpath(root); setup();
if nargin<2, overrides=struct(); end
cfg=nexarm.internal.mergeConfig(overrides);
if nargin>=1 && ~isempty(mode), cfg.mode=mode; end
cfg.mode=validatestring(cfg.mode,{'off','pid','lqr','mpc'});
data=nexarm.trajectory.loadReference(); result=nexarm.simulation.run(data,cfg);
runs=struct(); runs.(cfg.mode)=result; summary=nexarm.metrics.summaryTable(runs); disp(summary);
if cfg.save_results
    if ~isfolder(cfg.output_dir), mkdir(cfg.output_dir); end
    save(fullfile(cfg.output_dir,[cfg.mode '.mat']),'result','data','summary');
    writetable(summary,fullfile(cfg.output_dir,[cfg.mode '_summary.csv']));
end
if cfg.plot
    fig=nexarm.visualization.plotComparison(data,runs);
    if cfg.save_results, exportgraphics(fig,fullfile(cfg.output_dir,[cfg.mode '.png']),'Resolution',160); end
end
if cfg.animate, nexarm.visualization.animate(data,result); end
end
