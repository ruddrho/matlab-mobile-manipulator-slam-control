function L=mm_scan_hold(q,base,t0,c)
% Stationary six-second geometric inspection, visual sweep rendered separately.
n=round(c.scanDuration/c.dt);L.t=t0+(1:n)'*c.dt;
L.base=repmat(base,n,1);L.q=repmat(q',n,1);L.agv=repmat(c.agvPark,n,1);
d=min(mm_clearance(base,c.agvPark,c),mm_arm_clearance(q,base,c.agvPark,c));
L.clearance=ones(n,1)*d;L.speed=zeros(n,2);L.front=nan(n,1);
L.pathError=nan(n,1);L.mode=ones(n,1)*4;L.scanProgress=(1:n)'/n;
end
