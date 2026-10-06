function [q,qd,qdd]=mm_quintic(q0,q1,t,T)
u=max(0,min(1,t/T));d=q1-q0;
s=10*u^3-15*u^4+6*u^5;
sd=(30*u^2-60*u^3+30*u^4)/T;
sdd=(60*u-180*u^2+120*u^3)/(T*T);
q=q0+d*s;qd=d*sd;qdd=d*sdd;
end
