function [base,log,agvState]=mm_drive(base,goal,t0,c,agvState)
path=mm_plan_route(base(1:2),goal(1:2),c);v=0;w=0;t=0;
log.t=[];log.base=[];log.q=[];log.agv=[];log.clearance=[];
log.speed=[];log.front=[];log.pathError=[];log.mode=[];log.path=path;
idx=1;progress=[0;cumsum(sqrt(sum(diff(path).^2,2)))];
events=mm_crossings(path,c);nextEvent=1;activeEvent=0;startCross=0;
log.agvActive=[];log.crossingId=[];log.crossings=events;
if ~isfield(agvState,'count'),agvState.count=0;end
for k=1:10000
 dist=norm(base(1:2)-goal(1:2));
 % Local forward search prevents jumps to later sections of a curved route.
 search=idx:min(idx+45,size(path,1));
 [~,ii]=min(sum((path(search,:)-base(1:2)).^2,2));idx=max(idx,search(ii));
 if nextEvent<=numel(events) && activeEvent==0 && progress(idx)>=events(nextEvent).trigger
  activeEvent=nextEvent;nextEvent=nextEvent+1;startCross=t;
  agvState.count=agvState.count+1;
 end
 crossing=false;crossId=0;
 if activeEvent>0
  e=events(activeEvent);travel=c.agvSpeed*(t-startCross);
  agv=e.origin+e.normal*min(2*c.crossingHalfWidth,travel);
  crossing=travel<2*c.crossingHalfWidth;
  if crossing,crossId=agvState.count;else,activeEvent=0;end
 else,agv=c.agvPark;end
 agvState.active=crossing;
 [~,~,front]=mm_lidar(base,agv,c);
 idxLook=min(idx+5,size(path,1));target=path(idxLook,:);
 alpha=mm_wrap(atan2(target(2)-base(2),target(1)-base(1))-base(3));
 vc=min(c.vmax,1.1*dist)*max(0,cos(alpha));wc=max(-c.wmax,min(c.wmax,2.5*alpha));
 if dist<.04
  vc=0;wc=max(-c.wmax,min(c.wmax,2*mm_wrap(goal(3)-base(3))));
 end
 % Lidar field and time-to-crossing braking supervisor, with conservative
 % geometric predicted AGV clearance (known scripted AGV velocity).
 stopDist=c.margin+abs(v)*c.reaction+v*v/(2*c.brake);
 mode=0;
 if front<stopDist+.35,vc=min(vc,c.vmax*max(0,(front-stopDist)/.35));mode=1;end
 if front<stopDist,vc=0;mode=2;end
 % Explicit crossing interlock: brake to zero and hold until the AGV
 % has traversed the complete clear lane. Rendering shows STOP at this time.
 if crossing,vc=0;wc=0;mode=2;end
 rate=c.accel;if vc<v,rate=c.brake;end
 v=v+max(-rate*c.dt,min(rate*c.dt,vc-v));w=w+max(-3*c.dt,min(3*c.dt,wc-w));
 candidate=base+[v*cos(base(3)) v*sin(base(3)) w]*c.dt;candidate(3)=mm_wrap(candidate(3));
 if mm_clearance(candidate,agv,c)<c.margin*.5 || ...
    mm_station_clearance(base(1:2),candidate(1:2),c)<c.stationMargin
  candidate=base+[0 0 w]*c.dt;v=0;mode=2;
 end
 base=candidate;t=t+c.dt;
 clearance=min(mm_clearance(base,agv,c),mm_arm_clearance(c.arm.qHome,base,agv,c));
 log.agvActive(end+1,1)=crossing;log.crossingId(end+1,1)=crossId;
 log.t(end+1,1)=t0+t;log.base(end+1,:)=base;log.q(end+1,:)=c.arm.qHome';
 log.agv(end+1,:)=agv;log.clearance(end+1,1)=clearance;
 log.speed(end+1,:)=[v w];log.front(end+1,1)=front;
 log.pathError(end+1,1)=min(sqrt(sum((path-base(1:2)).^2,2)));log.mode(end+1,1)=mode;
 if clearance<0,error('mm:Collision','Collision in navigation at t=%.2f',t0+t);end
 if dist<.04 && abs(mm_wrap(goal(3)-base(3)))<.025 && abs(v)<.02,return;end

end
error('mm:NavigationTimeout','Navigation timed out; inspect scene/planner.');
end
