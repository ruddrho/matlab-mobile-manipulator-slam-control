function test_station_clearance()
here=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(here,'src'));
S=load(fullfile(here,'results','mission.mat'));m=S.mission;c=m.config;
minimum=inf;
% Dense independent sweep check covers translations, turns, and arm stages.
for k=2:size(m.base,1)
 for t=linspace(0,1,5)
  p=(1-t)*m.base(k-1,1:2)+t*m.base(k,1:2);
  for r=c.stationPads'
   q=max(r(1:2)',min(r(3:4)',p));d=norm(p-q)-c.baseRadius;
   minimum=min(minimum,d);assert(d>=c.stationMargin-1e-9,'Base crossed station safety buffer.');
  end
 end
end
assert(~mm_segment_free([6.1 3.8],[8 3.8],c),'Old pad-crossing shortcut must be rejected.');
assert(mm_station_clearance([6.83 3.8],[6.83 3.8],c)<0);
assert(m.completed && m.inspection.passed && all(m.placeError<.015));
fprintf('PASS: pickup/place pads avoided throughout complete mission; minimum base-envelope/pad clearance %.6f m.\n',minimum);
end
