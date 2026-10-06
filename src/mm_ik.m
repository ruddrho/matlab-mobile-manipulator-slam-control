function [q,err] = mm_ik(T,a,seed)
% Closed form position plus X-Y-X wrist decomposition; branch nearest seed.
R=T(1:3,1:3); w=T(1:3,4)-R*[0;0;a.tool];
r=hypot(w(1),w(2)); z=w(3)-a.h; L=a.L;
c3=(r*r+z*z-L(1)^2-L(2)^2)/(2*L(1)*L(2));
if abs(c3)>1+1e-10, error('mm:IKReach','Target outside arm workspace.'); end
c3=max(-1,min(1,c3)); q1=atan2(w(2),w(1)); candidates=[];
for elbow=[1 -1]
 q3=elbow*acos(c3);
 q2=atan2(-z,r)-atan2(L(2)*sin(q3),L(1)+L(2)*cos(q3));
 W=mm_rot([0;1;0],-q2-q3)*mm_rot([0;0;1],-q1)*R;
 b=acos(max(-1,min(1,W(1,1))));
 if abs(sin(b))<1e-8
  if W(1,1)>0, abc=[0;0;atan2(W(3,2),W(2,2))];
  else, abc=[0;pi;atan2(-W(3,2),W(2,2))]; end
  candidates=[candidates mm_wrap([q1;q2;q3;abc])]; %#ok<AGROW>
 else
  aa=atan2(W(2,1),-W(3,1)); cc=atan2(W(1,2),W(1,3));
  candidates=[candidates mm_wrap([q1;q2;q3;aa;b;cc]) ...
    mm_wrap([q1;q2;q3;aa+pi;-b;cc+pi])]; %#ok<AGROW>
 end
end
valid=all(candidates>=a.limit(:,1)-1e-9 & candidates<=a.limit(:,2)+1e-9,1);
if ~any(valid), error('mm:IKLimits','No joint-limit-feasible IK branch.');end
candidates=candidates(:,valid);
[~,i]=min(sum(mm_wrap(candidates-seed(:)).^2,1));q=candidates(:,i);
F=mm_fk(q,a);err=norm(F-T,'fro');
if err>1e-6,error('mm:IKResidual','IK residual is too large.');end
end
