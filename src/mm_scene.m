function mm_scene(m,k,ax)
axes(ax);hold(ax,'on');
c=m.config;base=m.base(k,:);q=m.q(k,:)';agv=m.agv(k,:);B=mm_rot([0;0;1],base(3));
colSilver=[.68 .72 .77];blue=[.02 .43 .82];black=[.055 .07 .10];
fresh=~isappdata(ax,'mm_static');
if fresh
setappdata(ax,'mm_reuse_active',false);
[X,Y]=meshgrid(c.world(1):.5:c.world(2),c.world(3):.5:c.world(4));
mesh(ax,X,Y,zeros(size(X)),'EdgeColor',[.85 .87 .9],'FaceColor',[.975 .98 .99]);
for j=1:numel(m.paths),p=m.paths{j};plot3(p(:,1),p(:,2),ones(size(p,1),1)*.015,'Color',[.08 .40 .85],'LineWidth',2.0);end

% dock, pickup pad, placement pad
box([-0.48 0 .50],[.15 1.18 1.0],[.7 .73 .78],eye(3));
box([-.38 0 .61],[.025 .88 .18],[.16 .23 .30],eye(3));
box([.98 0 .035],[.52 .80 .07],[.42 .84 .4],eye(3));
box([6.83 3.8 .035],[.56 .62 .07],[.98 .80 .10],eye(3));
text(-.65,.1,1.15,'CHARGING DOCK','FontSize',8);text(.7,-.6,.08,'PICKUP','FontSize',8);text(6.7,4.3,.08,'STACK / INSPECT','FontSize',8);
for j=1:size(c.obstacles,1)
 o=c.obstacles(j,:);kind=c.obstacleTypes{j};
 if strcmp(kind,'drum')
  cylinderBetween([o(1) o(2) 0],[o(1) o(2) o(4)],o(3),blue);
  for z=o(4)*[.08 .17 .80 .91],cylinderBetween([o(1) o(2) z],[o(1) o(2) z+.025],o(3)*1.03,[.03 .25 .48]);end
 elseif strcmp(kind,'cone')
  [xx,yy,zz]=cylinder([o(3) .025],24);surf(xx+o(1),yy+o(2),zz*o(4),'FaceColor',[1 .38 .04],'EdgeColor','none');
  box([o(1:2) .035],[.6 .6 .07],black,eye(3));
 elseif strcmp(kind,'carton')
  box([o(1:2) o(4)/2],[o(3)*1.4 o(3)*1.4 o(4)],[.72 .53 .27],eye(3));
  box([o(1:2) o(4)+.006],[.045 o(3)*1.42 .014],[.93 .80 .49],eye(3));
  box([o(1) o(2)-o(3)*.71-.005 o(4)*.55],[o(3)*.65 .012 .20],[.96 .91 .75],eye(3));
 else
  box([o(1:2) o(4)/2],[o(3)*1.4 o(3)*1.4 o(4)],[.96 .65 .07],eye(3));
  for zz=o(4)*[.22 .7]
   box([o(1) o(2)-o(3)*.71-.006 zz],[o(3)*1.4 .012 .06],black,eye(3));
  end
 end
end
% Mark crossing lanes once using the planner's verified lane geometry.
if isfield(m,'crossings')
 for gi=1:numel(m.crossings)
  ev=m.crossings{gi};
  for ei=1:numel(ev)
   v=ev(ei);aa=v.origin;bb=aa+2*c.crossingHalfWidth*v.normal;
   plot3([aa(1) bb(1)],[aa(2) bb(2)],[.028 .028],'--','Color',[.96 .52 .08],'LineWidth',1.3);
  end
 end
end
setappdata(ax,'mm_static',true);
end
setappdata(ax,'mm_reuse_active',true);setappdata(ax,'mm_cursor',0);
% Bound the displayed trail to 400 points.
ii=unique(round(linspace(1,k,min(k,400))));
reuseLine(m.base(ii,1),m.base(ii,2),ones(size(ii))*.02,'Color',[0 .50 .33],'LineWidth',1.5);
% Yellow crossing AGV
box([agv .25],[.56 .60 .45],[.98 .76 .03],eye(3));
box([agv(1) agv(2)-.305 .3],[.45 .012 .08],black,eye(3));
reuseObject('text','Position',[agv .58],'String','AGV','FontSize',8);
% Planar LiDAR safety field
ang=linspace(-pi/3,pi/3,32)+base(3);rr=.95;
sv=[base(1) base(2) .025; (base(1)+rr*cos(ang))' (base(2)+rr*sin(ang))' ones(32,1)*.025];
sf=[ones(31,1) (2:32)' (3:33)'];
reuseObject('patch','Vertices',sv,'Faces',sf,'FaceColor',[1 .24 .2],'FaceAlpha',.10,'EdgeColor','none');
[range,xy]=mm_lidar(base,agv,c,false);
jj=1:10:numel(range);
xx=[repmat(base(1),1,numel(jj));xy(1,jj);nan(1,numel(jj))];
yy=[repmat(base(2),1,numel(jj));xy(2,jj);nan(1,numel(jj))];
zz=repmat([.30;.30;NaN],1,numel(jj));
reuseLine(xx(:),yy(:),zz(:),'Color',[.78 .86 .92],'LineWidth',.25);
% Four wheels, silver skid-steer chassis, blue trim, emergency button
box([base(1:2) .31],[.84 .59 .33],colSilver,B);
box([base(1:2) .49],[.80 .55 .025],[.87 .89 .91],B);
for side=[-1 1]
 for along=[-.28 .28]
  pa=B*[along;side*.32;.19]+[base(1);base(2);0];
  pb=B*[along;side*.41;.19]+[base(1);base(2);0];
  cylinderBetween(pa',pb',.17,black);
  pc=pb+B*[0;side*.008;0];cylinderBetween(pb',pc',.12,blue);
  for a=[0 pi/2]
   d=B*[.12*cos(a);0;.12*sin(a)];reuseLine([pc(1)-d(1) pc(1)+d(1)],[pc(2)-d(2) pc(2)+d(2)],[pc(3)-d(3) pc(3)+d(3)],'Color',colSilver,'LineWidth',3);
  end
 end
 pt=B*[0;side*.302;.33]+[base(1);base(2);0];box(pt',[.49 .013 .075],blue,B);
end
pt=B*[.27;-.15;.52]+[base(1);base(2);0];cylinderBetween(pt',pt'+[0 0 .025],.045,[.75 .01 .02]);
% Six-joint industrial-style arm, joint hubs and two-finger gripper
[~,~,p,~,A]=mm_fk(q,c.arm);p=B*p+[base(1);base(2);0];A=B*A;
cylinderBetween([base(1:2) .49],[base(1:2) .57],.15,black);
for j=1:6
 if norm(p(:,j+1)-p(:,j))>.03
  start=p(:,j);if j==1,start(3)=.52;end
  cylinderBetween(start',p(:,j+1)',.065,colSilver);
 end
 if j>1
  pa=p(:,j)-.075*A(:,j);pb=p(:,j)+.075*A(:,j);
  cylinderBetween(pa',pb',.093,black);
  cylinderBetween(pb',pb'+.012*A(:,j)',.065,blue);
 end
end
T=mm_fk(q,c.arm);R=B*T(1:3,1:3);tcp=p(:,7);
box((tcp-R*[0;0;.06])',[.14 .10 .08],black,R);
for s=[-1 1],box((tcp+R*[s*.075;0;0])',[.025 .045 .13],colSilver,R);end
blocks=reshape(m.blocks(k,:),3,2)';colors=[.04 .38 .94;.91 .035 .045];
for j=1:2,box(blocks(j,:),c.blockSize,colors(j,:),eye(3));end
% Visible synthetic scan: sweep both stacked block faces and a wrist beam.
progress=0;if isfield(m,'scanProgress'),progress=m.scanProgress(k);end
scanning=progress>0;verts=nan(3,3);sx=NaN;sy=NaN;sz=NaN;label='';
if scanning
 center=mean(c.stack(:,1:2),1);height=.07+.40*progress;
 a=[center(1)-.13 center(2)-.105 height];b=[center(1)+.13 center(2)-.105 height];
 verts=[tcp';a;b];sx=[a(1) b(1)];sy=[a(2) b(2)];sz=[height height];
 label=sprintf('SCANNING RED + BLUE  %d%%',round(100*progress));
end
reuseObject('patch','Vertices',verts,'Faces',[1 2 3],'FaceColor',[.08 1 .40],'FaceAlpha',.18,'EdgeColor','none');
reuseLine(sx,sy,sz,'Color',[.02 1 .30],'LineWidth',3);
reuseObject('text','Position',[c.stack(1,1) c.stack(1,2) 1.40],'String',label,'Color',[.02 .55 .15],'FontWeight','bold','FontSize',10);
stopLabel='';if isfield(m,'agvActive') && m.agvActive(k),stopLabel='STOP | AGV CROSSING';end
reuseObject('text','Position',[base(1:2) 1.75],'String',stopLabel,'Color',[.9 .08 .04],'FontWeight','bold','FontSize',10);
if fresh
axis equal;axis([c.world 0 2]);view(42,28);xlabel('x (m)');ylabel('y (m)');zlabel('z (m)');
set(ax,'Color',[.975 .98 .99],'FontSize',9);grid on;
try,camlight('headlight');lighting gouraud;catch,end
end
end
function box(center,sizev,col,R)
v=[-1 -1 -1;1 -1 -1;1 1 -1;-1 1 -1;-1 -1 1;1 -1 1;1 1 1;-1 1 1].*sizev/2;
v=(R*v')'+center;quad=[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8];
faces=[quad(:,[1 2 3]);quad(:,[1 3 4])];
reuseObject('patch','Vertices',v,'Faces',faces,'FaceColor',col,'EdgeColor',col*.58,'LineWidth',.5);
end
function cylinderBetween(p1,p2,r,col)
d=p2(:)-p1(:);h=norm(d);if h<1e-8,return;end
z=d/h;ref=[1;0;0];if abs(dot(ref,z))>.9,ref=[0;1;0];end
x=cross(ref,z);x=x/norm(x);y=cross(z,x);R=[x y z];
[X,Y,Z]=cylinder(r,18);V=R*[X(:)';Y(:)';Z(:)'*h]+p1(:);
reuseObject('surface','XData',reshape(V(1,:),size(X)),'YData',reshape(V(2,:),size(X)),'ZData',reshape(V(3,:),size(X)), ...
 'FaceColor',col,'EdgeColor','none');
VX=reshape(V(1,:),size(X));VY=reshape(V(2,:),size(X));VZ=reshape(V(3,:),size(X));
for i=[1 2]
 verts=[mean(VX(i,1:end-1)) mean(VY(i,1:end-1)) mean(VZ(i,1:end-1));VX(i,:)' VY(i,:)' VZ(i,:)'];
 faces=[ones(18,1) (2:19)' (3:20)'];
 reuseObject('patch','Vertices',verts,'Faces',faces,'FaceColor',col,'EdgeColor','none');
end
end

function h=reuseLine(x,y,z,varargin)
h=reuseObject('line','XData',x,'YData',y,'ZData',z,varargin{:});
end
function h=reuseObject(kind,varargin)
% Per-axes pool. Every frame visits the same geometry in the same order.
ax=gca;
if ~isappdata(ax,'mm_reuse_active') || ~getappdata(ax,'mm_reuse_active')
 h=feval(kind,'Parent',ax,varargin{:});return;
end
n=getappdata(ax,'mm_cursor')+1;setappdata(ax,'mm_cursor',n);
pool=getappdata(ax,'mm_pool');if isempty(pool),pool={};end
if n<=numel(pool) && isgraphics(pool{n})
 h=pool{n};set(h,varargin{:});
else
 h=feval(kind,'Parent',ax,varargin{:});pool{n}=h;setappdata(ax,'mm_pool',pool);
end
end
