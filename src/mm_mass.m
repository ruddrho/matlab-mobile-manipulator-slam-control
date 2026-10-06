function M=mm_mass(q,a,payload)
M=zeros(6);z=zeros(6,1);
for j=1:6,e=z;e(j)=1;M(:,j)=mm_rne(q,z,e,a,payload,[0;0;0]);end
M=(M+M')/2;
end
