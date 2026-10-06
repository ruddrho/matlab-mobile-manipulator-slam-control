function events=mm_crossings(path,c)
% Select crossing lanes that clear static geometry and world boundaries.
% Each lane is perpendicular to the local route; trigger upstream.
s=[0;cumsum(sqrt(sum(diff(path).^2,2)))];events=struct([]);
if s(end)<6,return;end
for fraction=c.crossingFractions
 target=fraction*s(end);[~,order]=sort(abs(s-target));
 for i=order'
  if s(i)<2.2 || s(i)>s(end)-2.2,continue;end
  if ~isempty(events) && min(abs([events.s]-s(i)))<4,continue;end
  lo=max(1,i-3);hi=min(size(path,1),i+3);v=path(hi,:)-path(lo,:);v=v/norm(v);
  normal=[-v(2) v(1)];a=path(i,:)-c.crossingHalfWidth*normal;b=path(i,:)+c.crossingHalfWidth*normal;
  ca=c;ca.baseRadius=c.agvRadius;ca.margin=.12;
  inside=all(min([a;b],[],1)>[c.world(1) c.world(3)]+c.agvRadius+.05) && ...
   all(max([a;b],[],1)<[c.world(2) c.world(4)]-c.agvRadius-.05);
  if inside && mm_segment_free(a,b,ca)
   e.center=path(i,:);e.normal=normal;e.origin=a;e.s=s(i);
   e.trigger=max(0,s(i)-c.crossingLead);e.index=i;
   events=[events e];break; %#ok<AGROW>
  end
 end
end
if numel(events)<numel(c.crossingFractions)
 error('mm:CrossingLane','Found %d of %d clear crossing lanes.',numel(events),numel(c.crossingFractions));
end
[~,order]=sort([events.trigger]);events=events(order);
end
