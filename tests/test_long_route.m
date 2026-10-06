function test_long_route()
here=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(here,'src'));
S=load(fullfile(here,'results','mission.mat'));m=S.mission;c=m.config;
assert(m.completed && m.collisionSamples==0);
assert(m.routeLength>65,'Mission must include the extended route.');
assert(m.agvCrossingCount==8,'Expected two crossings on each of four long legs.');
for j=1:8
 ii=find(m.crossingId==j);assert(~isempty(ii));
 stable=ii(m.t(ii)>m.t(ii(1))+1);
 assert(~isempty(stable) && max(abs(m.speed(stable,1)))<1e-8,'Base failed to stop for crossing.');
 assert(max(abs(m.speed(stable,2)))<1e-8,'Base kept rotating during crossing.');
end
scan=find(m.scanProgress>0);assert(numel(scan)==round(c.scanDuration/c.dt));
assert(all(m.held(scan)==0) && all(m.mode(scan)==4));
scanSpeed=m.speed(scan,:);assert(max(abs(scanSpeed(:)))<1e-10);
lastBlocks=reshape(m.blocks(scan(end),:),3,2)';assert(max(sqrt(sum((lastBlocks-c.stack).^2,2)))<.015);
for k=1:numel(m.paths)
 path=m.paths{k};
 for i=2:size(path,1),assert(mm_segment_free(path(i-1,:),path(i,:),c));end
end
fprintf('PASS: extended route, eight AGV full stops, collision-free paths, and six-second scan after both boxes placed.\n');
end
