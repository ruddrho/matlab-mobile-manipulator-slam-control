function [q,log]=mm_stow(q,base,t0,payload,c)
% Home orientation is downward; use the same validated servo as manipulation.
F=mm_fk(c.arm.qHome,c.arm);B=mm_rot([0;0;1],base(3));
target=B*F(1:3,4)+[base(1);base(2);0];
% qHome has identity wrist orientation, so perform a separate joint move.
q0=q;dt=c.armDt;duration=3;n=round(3.6/dt);
log.t=[];log.base=[];log.q=[];log.agv=[];log.clearance=[];log.speed=[];log.front=[];log.pathError=[];log.mode=[];
qd=zeros(6,1);
for k=1:n
 [qr,vr,ar]=mm_quintic(q0,c.arm.qHome,k*dt,duration);
 qdd=max(-8,min(8,ar+c.arm.Kp.*mm_wrap(qr-q)+c.arm.Kd.*(vr-qd)));
 qd=max(-1.5,min(1.5,qd+qdd*dt));q=q+qd*dt;
 tau=mm_rne(q,qd,qdd,c.arm,payload);
 if any(abs(tau)>c.arm.torqueLimit*1.05),error('mm:Torque','Stow torque exceeded.');end
 if mod(k,round(c.dt/dt))==0||k==n
  agv=c.agvPark;cl=min(mm_clearance(base,agv,c),mm_arm_clearance(q,base,agv,c));
  if cl<0,error('mm:ArmCollision','Stow collision.');end
  log.t(end+1,1)=t0+k*dt;log.base(end+1,:)=base;log.q(end+1,:)=q';log.agv(end+1,:)=agv;
  log.clearance(end+1,1)=cl;log.speed(end+1,:)=[0 0];log.front(end+1,1)=NaN;
  log.pathError(end+1,1)=NaN;log.mode(end+1,1)=3;
 end
end
if norm(mm_wrap(q-c.arm.qHome))>.02,error('mm:Stow','Stow tracking failed.');end
end
