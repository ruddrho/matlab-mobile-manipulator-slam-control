function [q,log]=mm_arm_move(q0,target,base,t0,payload,c,cartesian)
% Tracking through a second-order joint servo; RNE evaluates required torque.
% The separate experiments integrate torque-driven rigid-body dynamics.
if nargin<7,cartesian=false;end
T=eye(4);T(1:3,1:3)=diag([1 -1 -1]);
B=mm_rot([0;0;1],base(3));T(1:3,4)=B'*(target(:)-[base(1);base(2);0]);
q1=mm_ik(T,c.arm,q0);duration=3;dt=c.armDt;
q=q0;qd=zeros(6,1);n=round(duration/dt)+round(.6/dt);
log.t=[];log.base=[];log.q=[];log.agv=[];log.clearance=[];log.speed=[];
log.front=[];log.pathError=[];log.mode=[];log.tau=[];log.error=[];log.sigma=[];
F0=mm_fk(q0,c.arm);prev=q0;prevD=zeros(6,1);
for k=1:n
 tt=k*dt;[qr,vr,ar]=mm_quintic(q0,q1,tt,duration);
 if cartesian
  u=min(1,tt/duration);s=10*u^3-15*u^4+6*u^5;
  Tc=T;Tc(1:3,4)=F0(1:3,4)+(T(1:3,4)-F0(1:3,4))*s;
  qr=mm_ik(Tc,c.arm,prev);vr=mm_wrap(qr-prev)/dt;ar=(vr-prevD)/dt;
  if k==1,vr=zeros(6,1);ar=zeros(6,1);end
  prev=qr;prevD=vr;
 end
 qdd=ar+c.arm.Kp.*mm_wrap(qr-q)+c.arm.Kd.*(vr-qd);
 % Joint servo acceleration and velocity limits keep animation physical.
 qdd=max(-8,min(8,qdd));qd=max(-1.5,min(1.5,qd+qdd*dt));q=q+qd*dt;
 tau=mm_rne(q,qd,qdd,c.arm,payload);
 if any(abs(tau)>c.arm.torqueLimit*1.05),error('mm:Torque','Mission torque limit exceeded.');end
 if any(q<c.arm.limit(:,1))||any(q>c.arm.limit(:,2)),error('mm:JointLimit','Joint limit reached.');end
 if mod(k,round(c.dt/dt))==0||k==n
  [F,J]=mm_fk(q,c.arm);sval=svd(J);agv=c.agvPark;
  cl=min(mm_clearance(base,agv,c),mm_arm_clearance(q,base,agv,c));
  if cl<0,error('mm:ArmCollision','Arm/environment collision.');end
  log.t(end+1,1)=t0+tt;log.base(end+1,:)=base;log.q(end+1,:)=q';log.agv(end+1,:)=agv;
  log.clearance(end+1,1)=cl;log.speed(end+1,:)=[0 0];log.front(end+1,1)=NaN;
  log.pathError(end+1,1)=NaN;log.mode(end+1,1)=3;log.tau(end+1,:)=tau';
  log.error(end+1,1)=norm(F(1:3,4)-T(1:3,4));log.sigma(end+1,1)=min(sval);
 end
end
F=mm_fk(q,c.arm);
if norm(F(1:3,4)-T(1:3,4))>.015,error('mm:Tracking','Arm failed to settle.');end
end
