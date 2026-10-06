function path=mm_plan_route(start,goal,c)
% A* between mandatory workcell waypoints; validated rounded corners.
if norm(start(1:2)-goal(1:2))<3
 path=mm_astar(start(1:2),goal(1:2),c);return;
end
way=c.route;
if norm(goal(1:2)-c.goalBase(1:2))>.2,way=flipud(way);end
way=[start(1:2);way(2:end-1,:);goal(1:2)];poly=way(1,:);
for j=2:size(way,1)
 [~,v]=mm_astar(way(j-1,:),way(j,:),c);poly=[poly;v(2:end,:)]; %#ok<AGROW>
end
keep=[true;sqrt(sum(diff(poly).^2,2))>1e-7];poly=poly(keep,:);
path=poly(1,:);
for i=2:size(poly,1)-1
 a=poly(i-1,:);b=poly(i,:);d=poly(i+1,:);
 ra=norm(b-a);rd=norm(d-b);radius=min([c.cornerRadius .35*ra .35*rd]);
 accepted=false;
 for attempt=1:8
  enter=b+(a-b)*radius/ra;leave=b+(d-b)*radius/rd;
  u=linspace(0,1,17)';curve=(1-u).^2.*enter+2*(1-u).*u.*b+u.^2.*leave;
  v=[path(end,:);curve];safe=true;
  for k=2:size(v,1)
   if ~mm_segment_free(v(k-1,:),v(k,:),c),safe=false;break;end
  end
  if safe,path=[path;curve];accepted=true;break;end %#ok<AGROW>
  radius=radius/2;
 end
 if ~accepted,path=[path;b];end %#ok<AGROW>
end
path=mm_resample([path;poly(end,:)],.06);
for i=2:size(path,1)
 if ~mm_segment_free(path(i-1,:),path(i,:),c),error('mm:RouteCollision','Rounded route failed clearance check.');end
end
end
