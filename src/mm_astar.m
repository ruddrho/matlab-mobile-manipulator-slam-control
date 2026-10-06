function [path,corners]=mm_astar(start,goal,c)
% Eight-connected A*; prohibit diagonal corner cutting; inflate obstacles.
xs=c.world(1):c.grid:c.world(2);ys=c.world(3):c.grid:c.world(4);
[X,Y]=ndgrid(xs,ys);occ=false(size(X));pad=c.baseRadius+c.margin;
for o=c.obstacles',occ=occ|hypot(X-o(1),Y-o(2))<=o(3)+pad;end
for r=c.stationPads'
 dx=max(max(r(1)-X,0),X-r(3));dy=max(max(r(2)-Y,0),Y-r(4));
 occ=occ|hypot(dx,dy)<=c.baseRadius+c.stationMargin;
end
occ=occ|X<c.world(1)+pad|X>c.world(2)-pad|Y<c.world(3)+pad|Y>c.world(4)-pad;
[~,sx]=min(abs(xs-start(1)));[~,sy]=min(abs(ys-start(2)));
[~,gx]=min(abs(xs-goal(1)));[~,gy]=min(abs(ys-goal(2)));
s=sub2ind(size(X),sx,sy);g=sub2ind(size(X),gx,gy);
if occ(s)||occ(g),error('mm:BlockedEndpoint','Start/goal in inflated obstacle.');end
G=inf(size(X));score=G;parent=zeros(size(X));closed=occ;G(s)=0;
score(s)=hypot(sx-gx,sy-gy);moves=[1 0;-1 0;0 1;0 -1;1 1;1 -1;-1 1;-1 -1];
while true
 [v,k]=min(score(:));if isinf(v),error('mm:NoPath','A* found no path.');end
 if k==g,break;end
 score(k)=inf;closed(k)=true;[ix,iy]=ind2sub(size(X),k);
 for j=1:8
  nx=ix+moves(j,1);ny=iy+moves(j,2);
  if nx<1||ny<1||nx>size(X,1)||ny>size(X,2)||closed(nx,ny),continue;end
  if all(moves(j,:)~=0)&&(occ(ix,ny)||occ(nx,iy)),continue;end
  if ~mm_segment_free([xs(ix) ys(iy)],[xs(nx) ys(ny)],c),continue;end
  cost=G(k)+norm(moves(j,:));
  if cost<G(nx,ny)
   G(nx,ny)=cost;parent(nx,ny)=k;score(nx,ny)=cost+hypot(nx-gx,ny-gy);
  end
 end
end
idx=g;while idx(1)~=s,idx=[parent(idx(1)) idx];end %#ok<AGROW>
raw=[xs(mod(idx-1,size(X,1))+1)' ys(floor((idx-1)/size(X,1))+1)'];
raw=[start(1:2);raw;goal(1:2)];path=raw(1,:);i=1;
% Greedy shortcutting only when the whole segment clears inflated circles.
while i<size(raw,1)
 j=size(raw,1);
 while j>i+1 && ~mm_segment_free(raw(i,:),raw(j,:),c),j=j-1;end
 path=[path;raw(j,:)];i=j; %#ok<AGROW>
end
% Resample so lookahead and reported path distance do not depend on grid.
corners=path;path=mm_resample(path,0.08);
end
