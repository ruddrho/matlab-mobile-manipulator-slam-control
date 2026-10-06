function s=mm_slam_step(s,ranges,angles,odom,maxRange)
% Online scan-to-map localization + log-odds occupancy mapping.
% Input is range data and incremental wheel odometry, not obstacle geometry.
d=odom(1:2)-s.odom(1:2);a=s.pose(3)-s.odom(3);
R=[cos(a) -sin(a);sin(a) cos(a)];
pred=[s.pose(1:2)+(R*d')' s.pose(3)+mm_wrap(odom(3)-s.odom(3))];
ranges=ranges(:);angles=angles(:);valid=isfinite(ranges)&ranges>0;
ranges=ranges(valid);angles=angles(valid);hit=ranges<maxRange-.015;
local=[ranges.*cos(angles) ranges.*sin(angles)];
s.pose=pred;
if s.count>2 && nnz(hit)>8 && (norm(d)>.005 || abs(odom(3)-s.odom(3))>.005)
 % Coarse local correlative scan matcher with a weak odometry prior.
 map=max(s.L,0);map=conv2(map,[1 2 1;2 4 2;1 2 1]/16,'same');
 points=local(hit,:);points=points(1:2:end,:);best=-inf;
 for da=[-.025 0 .025]
  th=pred(3)+da;R=[cos(th) -sin(th);sin(th) cos(th)];xy=points*R';
  for dx=[-.04 0 .04]
   for dy=[-.04 0 .04]
    q=xy+pred(1:2)+[dx dy];[idx,inside]=cells(q,s);
    score=sum(map(idx(inside)))/size(points,1)-.06*((dx/.04)^2+(dy/.04)^2+(da/.025)^2);
    if score>best,best=score;s.pose=pred+[dx dy da];end
   end
  end
 end
end
th=s.pose(3);R=[cos(th) -sin(th);sin(th) cos(th)];xy=local*R'+s.pose(1:2);
% Trace measured beams. Unknown stays 0; hit endpoints receive positive odds.
free=[];
for j=1:2:numel(ranges)
 lengthFree=max(0,ranges(j)-s.res);steps=0:s.res*.7:lengthFree;
 beam=s.pose(1:2)+[cos(th+angles(j));sin(th+angles(j))]' .*steps';
 [idx,inside]=cells(beam,s);free=[free;idx(inside)]; %#ok<AGROW>
end
s.L(unique(free))=s.L(unique(free))-.35;
[idx,inside]=cells(xy(hit,:),s);idx=unique(idx(inside));s.L(idx)=s.L(idx)+.85;
s.L=max(-4,min(4,s.L));s.hits=xy(hit,:);s.trail(end+1,:)=s.pose(1:2);
s.odom=odom;s.count=s.count+1;
end
function [idx,inside]=cells(q,s)
x=round((q(:,1)-s.x(1))/s.res)+1;y=round((q(:,2)-s.y(1))/s.res)+1;
inside=x>=1 & x<=numel(s.x) & y>=1 & y<=numel(s.y);
x=max(1,min(numel(s.x),x));y=max(1,min(numel(s.y),y));idx=sub2ind(size(s.L),y,x);
end
