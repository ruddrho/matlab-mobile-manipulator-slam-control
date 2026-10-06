function r=mm_trial(c,scenario,controller,seed)
rng(seed);a=c.arm;plant=a;plant.mass=a.mass*scenario(2);plant.inertia=a.inertia*scenario(2);
payload=scenario(1);knownPayload=payload; % payload known, link mismatch not known
T=eye(4);T(1:3,1:3)=diag([1 -1 -1]);T(1:3,4)=[.73;.12;.35];
q0=a.qHome;q1=mm_ik(T,a,q0);dt=.005;t=(0:dt:5)';n=numel(t);
q=q0;qd=zeros(6,1);r.t=t;r.error=zeros(n,6);r.tau=zeros(n,6);r.q=zeros(n,6);sat=0;
for i=1:n
 [qr,vr,ar]=mm_quintic(q0,q1,t(i),4);
 qm=q+scenario(3)*randn(6,1);vm=qd+scenario(3)*3*randn(6,1);
 e=mm_wrap(qr-qm);
 if controller==1
  tau=[65 90 60 14 14 10]'.*e+[16 21 14 3 3 2.5]'.*(vr-vm)+mm_rne(qm,zeros(6,1),zeros(6,1),a,knownPayload);
 else
  acc=ar+a.Kp.*e+a.Kd.*(vr-vm);tau=mm_rne(qm,vm,acc,a,knownPayload);
 end
 sat=sat+sum(abs(tau)>a.torqueLimit);tau=max(-a.torqueLimit,min(a.torqueLimit,tau));
 disturbance=zeros(6,1);
 if scenario(2)~=1 && t(i)>2 && t(i)<2.5,disturbance(2)=2.0;end
 M=mm_mass(q,plant,payload);h=mm_rne(q,qd,zeros(6,1),plant,payload);
 r.error(i,:)=mm_wrap(qr-q)';r.tau(i,:)=tau';r.q(i,:)=q';
 if i<n
  qdd=M\(tau+disturbance-h);qd=qd+qdd*dt;q=q+qd*dt;
 end
 if any(~isfinite(q))||norm(qd)>100,error('mm:Unstable','Plant integration diverged.');end
end
names={'PD_gravity','Computed_torque'};r.controller=names{controller};r.case=scenario;
r.rms=sqrt(mean(r.error(:).^2));r.peak=max(abs(r.error(:)));r.effort=dt*sum(r.tau(:).^2);
r.saturation=sat/(n*6);r.final=norm(r.error(end,:));
end
