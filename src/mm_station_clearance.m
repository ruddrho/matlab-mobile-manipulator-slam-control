function clearance=mm_station_clearance(a,b,c)
% Exact swept circular-base clearance from closed rectangular station pads.
% Split at rectangle boundaries; squared distance is quadratic per interval.
a=a(:)';b=b(:)';v=b-a;best=inf;
for r=c.stationPads'
 lo=r(1:2)';hi=r(3:4)';ts=[0 1];
 for j=1:2
  if abs(v(j))>eps,ts=[ts (lo(j)-a(j))/v(j) (hi(j)-a(j))/v(j)];end %#ok<AGROW>
 end
 ts=unique(sort(ts(ts>=0 & ts<=1)));
 for k=1:numel(ts)-1
  l=ts(k);h=ts(k+1);mid=a+((l+h)/2)*v;
  low=mid<lo;high=mid>hi;active=low|high;
  bound=lo.*low+hi.*high;offset=(a-bound).*active;vel=v.*active;
  t=max(l,min(h,-dot(offset,vel)/max(dot(vel,vel),eps)));
  q=a+t*v;delta=max(max(lo-q,0),q-hi);
  best=min(best,norm(delta));
 end
end
clearance=best-c.baseRadius;
end
