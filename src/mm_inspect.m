function out=mm_inspect(q,base,blocks,c)
% Synthetic wrist pinhole camera. Camera looks along tool +Z (down at stack).
T=mm_fk(q,c.arm);B=mm_rot([0;0;1],base(3));R=B*T(1:3,1:3);
p=B*T(1:3,4)+[base(1);base(2);0];xyz=R'*(blocks'-p);
f=310;out.uv=[320+f*xyz(1,:)./xyz(3,:);240+f*xyz(2,:)./xyz(3,:)];
out.visible=all(xyz(3,:)>0 & abs(out.uv(1,:)-320)<320 & abs(out.uv(2,:)-240)<240);
out.alignment=norm(blocks(1,1:2)-blocks(2,1:2));
out.heightError=abs(blocks(2,3)-blocks(1,3)-c.blockSize(3));
out.passed=out.visible && out.alignment<.02 && out.heightError<.02;
out.note='Ideal projected known centroids; no image recognition or occlusion model.';
end
