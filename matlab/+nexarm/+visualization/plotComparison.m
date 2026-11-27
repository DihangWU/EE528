function fig=plotComparison(data,runs,visible)
%PLOTCOMPARISON Reference/actual ink, dynamic error, cumulative pen-down RMS.
if nargin<3, visible='on'; end
fig=figure('Name','NexArm controller comparison','Color','w','Visible',visible,...
    'Position',[100 100 1280 760]);
if isprop(fig,'Theme'), fig.Theme='light'; end
layout=tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');
path_ax=nexttile(layout,1,[2 1]); hold(path_ax,'on');
ideal=nexarm.visualization.strokeLine(data.reference_tips,data.ink)*1000;
plot(path_ax,ideal(:,2),-ideal(:,1),'Color',[.65 .65 .65],'LineWidth',2,'DisplayName','Reference');
error_ax=nexttile(layout,2); hold(error_ax,'on'); rms_ax=nexttile(layout,4); hold(rms_ax,'on');
colors=struct('off',[.95 .25 .12],'pid',[.1 .65 .25],'lqr',[.65 .3 .85],'mpc',[0 .5 .85]);
names=fieldnames(runs);
for k=1:numel(names)
    name=names{k}; result=runs.(name); color=colors.(name);
    actual=nexarm.visualization.strokeLine(result.tips,data.ink)*1000;
    plot(path_ax,actual(:,2),-actual(:,1),'Color',color,'LineWidth',.9,'DisplayName',upper(name));
    plot(error_ax,result.time,result.metrics.pen_error_mm,'Color',color,'DisplayName',upper(name));
    plot(rms_ax,result.time,result.metrics.running_rms_mm,'Color',color,'DisplayName',upper(name));
end
axis(path_ax,'equal'); grid(path_ax,'on'); xlabel(path_ax,'Y (mm)'); ylabel(path_ax,'-X (mm)');
title(path_ax,'Pen-down writing path | view from behind base'); legend(path_ax,'Location','best');
grid(error_ax,'on'); xlabel(error_ax,'Time (s)'); ylabel(error_ax,'3D tracking error (mm)');
title(error_ax,'Same-time error | includes pen-up motion');
grid(rms_ax,'on'); xlabel(rms_ax,'Time (s)'); ylabel(rms_ax,'Cumulative RMS (mm)');
title(rms_ax,'Pen-down samples only'); legend(rms_ax,'Location','best');
end
