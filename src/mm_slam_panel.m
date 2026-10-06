function h=mm_slam_panel(ax,s,h)
% Allocate once; update existing graphics during playback and video export.
if nargin<3
 h.map=imagesc(ax,s.x,s.y,ones(size(s.L))*.5,[0 1]);hold(ax,'on');
 colormap(ax,gray(256));set(ax,'YDir','normal');axis(ax,'equal');
 xlim(ax,s.x([1 end]));ylim(ax,s.y([1 end]));
 h.trail=plot(ax,NaN,NaN,'Color',[0 .55 .8],'LineWidth',1.2);
 h.hits=plot(ax,NaN,NaN,'.','Color',[.85 .25 .12],'MarkerSize',3);
 h.robot=plot(ax,NaN,NaN,'o','MarkerFaceColor',[1 .65 0],'MarkerEdgeColor','k','MarkerSize',6);
 h.heading=plot(ax,NaN,NaN,'Color',[1 .5 0],'LineWidth',2);
 title(ax,'LIVE SLAM MAP','FontSize',12);xlabel(ax,'Black: occupied | White: free | Gray: unknown','FontSize',8);
end
set(h.map,'CData',1-1./(1+exp(-s.L)));
ii=unique(round(linspace(1,size(s.trail,1),min(600,size(s.trail,1)))));
set(h.trail,'XData',s.trail(ii,1),'YData',s.trail(ii,2));
set(h.hits,'XData',s.hits(:,1),'YData',s.hits(:,2));
set(h.robot,'XData',s.pose(1),'YData',s.pose(2));
set(h.heading,'XData',s.pose(1)+[0 .45*cos(s.pose(3))],'YData',s.pose(2)+[0 .45*sin(s.pose(3))]);
end
