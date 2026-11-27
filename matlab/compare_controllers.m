function [summary,runs]=compare_controllers(overrides)
%COMPARE_CONTROLLERS Compare all modes with identical trajectory and noise.
root=fileparts(mfilename('fullpath')); addpath(root); setup();
if nargin<1, overrides=struct(); end
cfg=nexarm.internal.mergeConfig(overrides); data=nexarm.trajectory.loadReference();
errors=nexarm.simulation.disturbances(data,cfg); modes={'off','pid','lqr','mpc'}; runs=struct();
for k=1:numel(modes)
    cfg.mode=modes{k}; fprintf('Running %s ...\n',upper(cfg.mode));
    runs.(cfg.mode)=nexarm.simulation.run(data,cfg,errors);
end
summary=nexarm.metrics.summaryTable(runs); disp(summary);
if cfg.save_results
    if ~isfolder(cfg.output_dir), mkdir(cfg.output_dir); end
    save(fullfile(cfg.output_dir,'comparison.mat'),'runs','data','summary','cfg');
    writetable(summary,fullfile(cfg.output_dir,'comparison.csv'));
    text=jsonencode(table2struct(summary),'PrettyPrint',true);
    filename=fullfile(cfg.output_dir,'comparison.json');
    fid=fopen(filename,'w'); assert(fid>=0,'nexarm:OutputError','Cannot write %s',filename);
    cleanup=onCleanup(@() fclose(fid)); fprintf(fid,'%s\n',text);
end
if cfg.plot
    fig=nexarm.visualization.plotComparison(data,runs);
    if cfg.save_results, exportgraphics(fig,fullfile(cfg.output_dir,'comparison.png'),'Resolution',160); end
end
if cfg.animate, nexarm.visualization.animate(data,runs.mpc); end
end
