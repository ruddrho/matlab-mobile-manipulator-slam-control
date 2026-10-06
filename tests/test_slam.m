function test_slam()
here=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(here,'src'));
S=load(fullfile(here,'results','mission.mat'));m=S.mission;c=m.config;
s=mm_slam_init(c,m.base(1,:));assert(all(s.L(:)==0));
next=m.t(1);err=[];timer=tic;
for k=1:numel(m.t)
 if m.t(k)<next-1e-9,continue;end
 [r,~]=mm_lidar(m.base(k,:),m.agv(k,:),c,false);
 s=mm_slam_step(s,r,c.lidarAngles,m.base(k,:),c.lidarRange);
 err(end+1)=norm(s.pose(1:2)-m.base(k,1:2));next=next+s.period; %#ok<AGROW>
end
assert(s.count>2000 && all(isfinite(s.L(:))));
assert(nnz(s.L>1)>100 && nnz(s.L<-1)>1000);
assert(max(err)<.8,'Scan-matching drift exceeded test tolerance.');
fprintf('PASS: %d online scans; max position error %.3f m; processing %.2f s.\n',s.count,max(err),toc(timer));
save(fullfile(c.output,'slam_validation.mat'),'s','err','-v7');
f=figure('Visible','off');ax=axes('Parent',f);h=mm_slam_panel(ax,s);n=numel(findall(ax));
for k=1:5,h=mm_slam_panel(ax,s,h);assert(numel(findall(ax))==n);end
print(f,fullfile(c.output,'live_slam_map.png'),'-dpng','-r120');close(f);
fprintf('PASS: empty-map initialization, free/occupied observations, bounded pose error and stable map graphics.\n');
end
