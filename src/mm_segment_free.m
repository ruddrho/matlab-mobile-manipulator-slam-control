function ok=mm_segment_free(a,b,c)
% Exact closest-point test for an entire segment, using inflated circles.
a=a(:)';b=b(:)';d=b-a;obs=c.obstacles;
u=max(0,min(1,((obs(:,1:2)-a)*d')/max(dot(d,d),eps)));
nearest=a+u.*d;
ok=all(hypot(nearest(:,1)-obs(:,1),nearest(:,2)-obs(:,2))>obs(:,3)+c.baseRadius+c.margin);
if ok
 ok=mm_station_clearance(a,b,c)>c.stationMargin;
end
if ok
 lo=[c.world(1) c.world(3)]+c.baseRadius+c.margin;
 hi=[c.world(2) c.world(4)]-c.baseRadius-c.margin;
 ok=all(min([a;b],[],1)>=lo) && all(max([a;b],[],1)<=hi);
end
end
