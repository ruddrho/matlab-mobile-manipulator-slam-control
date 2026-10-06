function out=mm_resample(p,step)
out=p(1,:);
for k=2:size(p,1)
 n=max(1,ceil(norm(p(k,:)-p(k-1,:))/step));
 for j=1:n,out=[out;p(k-1,:)+(p(k,:)-p(k-1,:))*j/n];end %#ok<AGROW>
end
end
