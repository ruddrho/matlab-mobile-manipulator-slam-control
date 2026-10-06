function [T, J, p, Rall, axisWorld] = mm_fk(q,a)
% Six revolute joints: Z-Y-Y-X-Y-X; intersecting spherical wrist.
p=zeros(3,7); Rall=zeros(3,3,6); axisWorld=zeros(3,6); R=eye(3);
for i=1:6
 axisWorld(:,i)=R*a.axes(:,i);
 R=R*mm_rot(a.axes(:,i),q(i)); Rall(:,:,i)=R;
 p(:,i+1)=p(:,i)+R*a.offset(:,i);
end
T=[R p(:,7);0 0 0 1]; J=zeros(6);
for i=1:6
 J(:,i)=[cross(axisWorld(:,i),p(:,7)-p(:,i));axisWorld(:,i)];
end
end
