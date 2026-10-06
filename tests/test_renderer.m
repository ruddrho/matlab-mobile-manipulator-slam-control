function test_renderer()
% Run in desktop MATLAB; checks handle reuse and uniform playback timing.
here=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(here,'src'));
S=load(fullfile(here,'results','mission.mat'));m=S.mission;
r=mm_playback_log(m,30,2);assert(abs(r.t(end)-m.t(end))<1e-10);
assert(max(abs(diff(r.t(1:end-1))-2/30))<1e-10);
assert(max(abs(r.base(end,:)-m.base(end,:)))<1e-9);
f=figure('Visible','off');cleanup=onCleanup(@()close(f));ax=axes('Parent',f);
mm_scene(r,1,ax);pool=getappdata(ax,'mm_pool');count=numel(findall(ax));
indices=unique(round(linspace(2,numel(r.t),12)));
if isfield(r,'scanProgress'),indices=unique([indices find(r.scanProgress>0,1) find(r.scanProgress>0,1,'last')]);end
if isfield(r,'agvActive'),indices=unique([indices find(r.agvActive,1)]);end
for k=indices
 mm_scene(r,k,ax);assert(numel(findall(ax))==count,'Graphics objects accumulated.');
 assert(isequal(pool,getappdata(ax,'mm_pool')),'Objects were recreated.');
end
fprintf('PASS: stable scene handles and uniform playback timeline.\n');
end
