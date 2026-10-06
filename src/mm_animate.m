function output=mm_animate(m,record,fps,speed)
% Reuse scene/dashboard graphics. Live playback skips late frames; export
% writes every uniformly timed frame regardless of rendering throughput.
if nargin<2,record=false;end
if nargin<3,if record,fps=30;else,fps=25;end,end
if nargin<4,speed=2;end
source=m;
slam=mm_slam_init(m.config,m.base(1,:));scanIndex=1;nextScan=m.t(1);
m=mm_playback_log(m,fps,speed);c=m.config;local=mm_config();
c.output=local.output;if ~exist(c.output,'dir'),mkdir(c.output);end
output='';
f=figure('Name','Mobile Manipulator | 3D Simulation', ...
 'NumberTitle','off','WindowStyle','normal', ...
 'Color',[.96 .97 .99],'Position',[40 40 1280 720],'Resize','on');
ax=axes('Parent',f,'Position',[.045 .08 .91 .83]);
dashboard=figure('Name','Mobile Manipulator | Live SLAM and Dashboard', ...
 'NumberTitle','off','WindowStyle','normal', ...
 'Color',[.96 .97 .99],'Position',[140 70 820 720],'Resize','on');
a1=axes('Parent',dashboard,'Position',[.09 .25 .36 .15]);
hv=plot(a1,NaN,NaN,'Color',[0 .48 .8],'LineWidth',1.4);grid(a1,'on');
xlim(a1,[m.t(1) m.t(end)]);ylim(a1,[0 .55]);title(a1,'Base speed (m/s)');
a2=axes('Parent',dashboard,'Position',[.58 .25 .36 .15]);
hc=plot(a2,NaN,NaN,'Color',[.1 .55 .35],'LineWidth',1.2);grid(a2,'on');
xlim(a2,[m.t(1) m.t(end)]);ylim(a2,[0 max(.2,max(m.clearance)*1.1)]);title(a2,'Clearance (m)');
mapAx=axes('Parent',dashboard,'Position',[.10 .50 .80 .44]);
mapHandles=mm_slam_panel(mapAx,slam);
a3=axes('Parent',dashboard,'Position',[.09 .035 .85 .13]);axis(a3,'off');
info=text(a3,0,1,'','FontName','FixedWidth','FontSize',10,'VerticalAlignment','top');
mm_scene(m,1,ax);heading=title(ax,'','FontSize',13);
% Fixed projection with a following camera; axes and graphics stay allocated.
xlim(ax,c.world(1:2));ylim(ax,c.world(3:4));zlim(ax,[0 2]);
set(ax,'CameraViewAngle',42,'CameraViewAngleMode','manual');
writer=[];
if record
 set(f,'Resize','off'); % Keep exported video frame dimensions fixed.
 profiles=VideoWriter.getProfiles();
 if any(strcmp({profiles.Name},'MPEG-4'))
  output=fullfile(c.output,'mobile_manipulator_smooth.mp4');writer=VideoWriter(output,'MPEG-4');
 else
  output=fullfile(c.output,'mobile_manipulator_smooth.avi');writer=VideoWriter(output,'Motion JPEG AVI');
 end
 writer.FrameRate=fps;writer.Quality=90;open(writer);
 cleanup=onCleanup(@()close(writer)); %#ok<NASGU>
end
n=numel(m.t);k=1;timer=tic;
while k<=n && isgraphics(f)
 if isgraphics(dashboard)
 % Consume fixed sensor timestamps even when display frames are skipped.
 while scanIndex<=numel(source.t) && source.t(scanIndex)<=m.t(k)+1e-9
  if source.t(scanIndex)>=nextScan-1e-9
   [ranges,~]=mm_lidar(source.base(scanIndex,:),source.agv(scanIndex,:),c,false);
   slam=mm_slam_step(slam,ranges,c.lidarAngles,source.base(scanIndex,:),c.lidarRange);
   nextScan=nextScan+slam.period;
  end
  scanIndex=scanIndex+1;
 end
 mapHandles=mm_slam_panel(mapAx,slam,mapHandles);
 end
 mm_scene(m,k,ax);
 center=[m.base(k,1:2) .65];
 if m.mode(k)==4,center=[c.stack(1,1:2) .55];end
 set(ax,'CameraTarget',center,'CameraPosition',center+[3.6 -4.8 3.7]);
 set(heading,'String',sprintf('LONG ROUTE MOBILE MANIPULATOR | %.1f s | %.1fx\n%s',m.t(k),speed,m.stage{k}));
 if isgraphics(dashboard)
 ii=unique(round(linspace(1,k,min(k,400))));
 set(hv,'XData',m.t(ii),'YData',m.speed(ii,1));
 set(hc,'XData',m.t(ii),'YData',m.clearance(ii));
 placed=sum(sqrt(sum((reshape(m.blocks(k,:),3,2)'-c.stack).^2,2))<.02);
 names={'CRUISE','SLOW','STOP: AGV / SAFETY','MANIPULATE','SCANNING STACK'};
 set(info,'String',sprintf('BLOCKS: %d / 2 placed | %s\nLive LiDAR mapping + scan matching\nSimulation odometry | 5 Hz map update',placed,names{m.mode(k)+1}));
 end
 drawnow;
 if ~isgraphics(f),break;end
 if record
  writeVideo(writer,getframe(f));
  if mod(k,150)==0,fprintf('Export: %.0f%%\n',100*k/n);end
  k=k+1;
 else
  if k==n,break;end
  remaining=k/fps-toc(timer);if remaining>0,pause(remaining);end
  k=min(n,max(k+1,1+floor(toc(timer)*fps)));
 end
end
if record
 if k>n,fprintf('Video saved: %s\n',output);
 else,warning('mm:ExportStopped','Figure closed early; saved video is incomplete.');end
end
end
