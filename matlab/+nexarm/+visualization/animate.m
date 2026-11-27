function fig=animate(data,result,speed,max_frames)
%ANIMATE Kinematic stick arm and rigid pen, displayed in world coordinates.
% This viewer uses the same FK as evaluation, not Robotics System Toolbox.
if nargin<3, speed=4; end
if nargin<4, max_frames=400; end
validateattributes(speed,{'numeric'},{'real','finite','scalar','positive'});
validateattributes(max_frames,{'numeric'},{'real','finite','scalar','integer','positive'});
fig=figure('Name',['NexArm writing: ' upper(result.mode)],'Color','w');
if isprop(fig,'Theme'), fig.Theme='light'; end
ax=axes(fig); hold(ax,'on');
offset=data.model.world_offset;
patch(ax,[0 .48 .48 0],[-.18 -.18 .18 .18],[0 0 0 0],[.94 .94 .94],'EdgeColor','none');
reference=nexarm.visualization.strokeLine(data.reference_tips+offset,data.ink);
plot3(ax,reference(:,1),reference(:,2),reference(:,3),'Color',[.65 .65 .65]);
arm=plot3(ax,nan,nan,nan,'-o','LineWidth',5,'Color',[.3 .4 .6],'MarkerSize',5);
pen=plot3(ax,nan,nan,nan,'Color',[1 .5 0],'LineWidth',3);
ink_line=plot3(ax,nan,nan,nan,'Color',[0 .5 .85],'LineWidth',1.2);
axis(ax,'equal'); axis(ax,[-.08 .5 -.23 .23 -.03 .45]); grid(ax,'on'); view(ax,90,45);
xlabel(ax,'World X (m)'); ylabel(ax,'World Y (m)'); zlabel(ax,'World Z (m)');
n=size(result.actual,1); stride=max(1,ceil(n/max_frames)); frames=unique([1:stride:n n]);
clock=tic;
for index=1:numel(frames)
    if ~isgraphics(fig), return; end
    k=frames(index);
    [tip,points]=nexarm.model.forwardKinematics(data.model,result.actual(k,:));
    points=points+offset; tip=tip+offset;
    set(arm,'XData',points(:,1),'YData',points(:,2),'ZData',points(:,3));
    p=[points(end,:);tip]; set(pen,'XData',p(:,1),'YData',p(:,2),'ZData',p(:,3));
    line=nexarm.visualization.strokeLine(result.tips(1:k,:)+offset,data.ink(1:k));
    set(ink_line,'XData',line(:,1),'YData',line(:,2),'ZData',line(:,3));
    title(ax,sprintf('%s | %.1f / %.1f s | error %.3f mm',upper(result.mode),result.time(k),result.time(end),result.metrics.pen_error_mm(k)));
    drawnow;
    delay=result.time(k)/speed-toc(clock);
    if delay>0, pause(delay); end
end
end
