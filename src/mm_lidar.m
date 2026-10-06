function [ranges,xy,front]=mm_lidar(pose,agv,c,noise)
% Ray-circle intersection plus world walls. Scanner at planar base center.
if nargin<4,noise=true;end
ang=c.lidarAngles+pose(3);ux=cos(ang);uy=sin(ang);
ranges=ones(size(ang))*c.lidarRange;
obs=[c.obstacles(:,1:3);agv(1:2) c.agvRadius];
for o=obs'
 dx=pose(1)-o(1);dy=pose(2)-o(2);b=dx*ux+dy*uy;
 disc=b.^2-(dx*dx+dy*dy-o(3)^2);hit=disc>=0;
 t=-b-sqrt(max(0,disc));hit=hit & t>=0;
 ranges(hit)=min(ranges(hit),t(hit));
end
for k=1:2
 u=ux;if k==2,u=uy;end
 bounds=c.world(2*k-1:2*k);
 for wall=bounds
  t=(wall-pose(k))./u;hit=t>=0 & isfinite(t);
  ranges(hit)=min(ranges(hit),t(hit));
 end
end
if noise,ranges=max(0,min(c.lidarRange,ranges+c.lidarNoise*randn(size(ranges))));end
xy=[pose(1)+ranges.*ux;pose(2)+ranges.*uy];
mask=abs(c.lidarAngles)<pi/3;front=min(ranges(mask))-c.baseRadius;
end
