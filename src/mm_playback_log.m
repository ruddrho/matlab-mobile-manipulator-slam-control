function r=mm_playback_log(m,fps,speed)
% Uniform simulation-time sampling, independent of renderer execution time.
validateattributes(fps,{'numeric'},{'scalar','real','finite','>=',1,'<=',120});
validateattributes(speed,{'numeric'},{'scalar','real','finite','positive'});
assert(numel(m.t)>1 && all(diff(m.t)>0),'Mission timestamps must increase.');
t0=m.t(1);t1=m.t(end);tt=t0+(0:ceil((t1-t0)*fps/speed))'*speed/fps;
tt=min(tt,t1);r=m;r.t=tt;
names={'base','q','agv','blocks','speed','clearance'};
for j=1:numel(names)
 name=names{j};v=m.(name);
 if strcmp(name,'base'),v(:,3)=unwrap(v(:,3));end
 r.(name)=interp1(m.t,v,tt,'linear');
end
idx=interp1(m.t,(1:numel(m.t))',tt,'previous');idx=round(idx);
r.mode=m.mode(idx);r.held=m.held(idx);r.stage=m.stage(idx);
if isfield(m,'scanProgress'),r.scanProgress=m.scanProgress(idx);end
if isfield(m,'agvActive'),r.agvActive=m.agvActive(idx);end
if isfield(m,'crossingId'),r.crossingId=m.crossingId(idx);end
end
