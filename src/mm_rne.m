function tau=mm_rne(q,qd,qdd,a,payload,gravity)
% World-coordinate recursive Newton-Euler. Payload is a point mass at TCP.
% Gravity vector is acceleration, e.g. [0;0;-9.81]. No base-arm coupling.
if nargin<6,gravity=[0;0;-9.81];end
[~,~,p,R,A]=mm_fk(q,a);
w=zeros(3,1); al=w; ao=w; F=zeros(3,6); N=F;
for i=1:6
 al=al+A(:,i)*qdd(i)+cross(w,A(:,i)*qd(i));
 w=w+A(:,i)*qd(i);
 rc=R(:,:,i)*a.com(:,i); rr=R(:,:,i)*a.offset(:,i);
 ac=ao+cross(al,rc)+cross(w,cross(w,rc));
 I=R(:,:,i)*a.inertia(:,:,i)*R(:,:,i)';
 F(:,i)=a.mass(i)*(ac-gravity);
 N(:,i)=I*al+cross(w,I*w)+cross(rc,F(:,i));
 ao=ao+cross(al,rr)+cross(w,cross(w,rr));
end
f=payload*(ao-gravity);n=zeros(3,1);tau=zeros(6,1);
for i=6:-1:1
 n=N(:,i)+n+cross(p(:,i+1)-p(:,i),f);
 f=F(:,i)+f; tau(i)=A(:,i)'*n;
end
tau=tau+a.rotor.*qdd+a.friction.*qd;
end
