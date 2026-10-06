function d=mm_arm_clearance(q,base,agv,c)
% Vectorized sampled link/environment clearance (not full self-collision).
[~,~,p]=mm_fk(q,c.arm);B=mm_rot([0;0;1],base(3));p=B*p+[base(1);base(2);0];
x=zeros(3,85);u=linspace(0,1,17);
for j=2:6,x(:,(j-2)*17+(1:17))=p(:,j)*(1-u)+p(:,j+1)*u;end
r=.045;d=min(x(3,:)-r);obs=[c.obstacles;agv(1:2) c.agvRadius .50];
for o=obs'
 eligible=x(3,:)-r<=o(4);
 if any(eligible),d=min(d,min(hypot(x(1,eligible)-o(1),x(2,eligible)-o(2))-o(3)-r));end
end
end
