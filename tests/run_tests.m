function run_tests()
here=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(here,'src'));c=mm_config();a=c.arm;rng(21);
% Verify FK/IK on diverse reachable poses, and Jacobian against finite difference.
for i=1:50
 q=[randn*.4;-.9+rand*.5;1+rand*.6;randn*.3;.4+rand*.5;randn*.3];
 [T,J]=mm_fk(q,a);[qi,e]=mm_ik(T,a,q);assert(e<1e-7);assert(norm(mm_wrap(qi-q))<1e-5);
 h=1e-6;Jn=zeros(3,6);
 for j=1:6,qp=q;qm=q;qp(j)=qp(j)+h;qm(j)=qm(j)-h;Tp=mm_fk(qp,a);Tm=mm_fk(qm,a);Jn(:,j)=(Tp(1:3,4)-Tm(1:3,4))/(2*h);end
 assert(norm(Jn-J(1:3,:),'fro')<1e-6);
 M=mm_mass(q,a,1);assert(min(eig(M))>0);assert(norm(M-M','fro')<1e-10);
 % Gravity torques equal numerical potential-energy gradient.
 gn=zeros(6,1);
 for j=1:6,qp=q;qm=q;qp(j)=qp(j)+h;qm(j)=qm(j)-h;gn(j)=(potential(qp,a,1)-potential(qm,a,1))/(2*h);end
 tau=mm_rne(q,zeros(6,1),zeros(6,1),a,1);assert(norm(gn-tau)<1e-5);
end
try,T=eye(4);T(1,4)=10;mm_ik(T,a,a.qHome);error('Expected IK rejection');catch err,assert(strcmp(err.identifier,'mm:IKReach'));end
[q,v,acc]=mm_quintic(zeros(6,1),ones(6,1),0,2);assert(norm(q)+norm(v)+norm(acc)==0);
[q,v,acc]=mm_quintic(zeros(6,1),ones(6,1),2,2);assert(norm(q-1)+norm(v)+norm(acc)<1e-12);
p=mm_astar(c.pickBase,c.goalBase,c);
for j=2:size(p,1),assert(mm_segment_free(p(j-1,:),p(j,:),c));end
% Noise-free LiDAR ray directly towards a known cylinder.
c2=c;c2.obstacles=[2 0 .4 1];c2.lidarAngles=0;c2.world=[-10 10 -10 10];
[r,~,~]=mm_lidar([0 0 0],[8 8],c2,false);assert(abs(r-1.6)<1e-9);
fprintf('PASS: 50 FK/IK, Jacobian, SPD mass, gravity gradient checks; workspace rejection; quintic boundaries; collision-free A*; LiDAR geometry.\n');
end
function U=potential(q,a,payload)
[T,~,p,R]=mm_fk(q,a);U=payload*9.81*T(3,4);
for i=1:6,pc=p(:,i)+R(:,:,i)*a.com(:,i);U=U+a.mass(i)*9.81*pc(3);end
end
