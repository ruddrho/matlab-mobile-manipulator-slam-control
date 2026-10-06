function mm_plots(m,c)
f=figure('Name','Mission evidence','Color','w','Position',[80 80 1250 780]);
subplot(2,3,1);hold on;
for i=1:numel(m.paths),p=m.paths{i};plot(p(:,1),p(:,2),'--','Color',[.7 .7 .7]);end
plot(m.base(:,1),m.base(:,2),'Color',[0 .45 .7],'LineWidth',1.5);
for o=c.obstacles',rectangle('Position',[o(1:2)'-o(3) 2*o(3) 2*o(3)],'Curvature',[1 1],'FaceColor',[.9 .6 .2]);end
axis equal;grid on;title('A* path and measured base motion');xlabel('x (m)');ylabel('y (m)');
subplot(2,3,2);plot(m.t,m.speed(:,1),'LineWidth',1.2);hold on;plot(m.t,m.speed(:,2),'LineWidth',1);grid on;xlabel('Time (s)');title('Base controls');legend('v (m/s)','omega (rad/s)');
subplot(2,3,3);plot(m.t,m.clearance,'LineWidth',1.3);hold on;plot([m.t(1) m.t(end)],[0 0],'r--');grid on;xlabel('Time (s)');ylabel('m');title('Minimum modeled clearance');
subplot(2,3,4);plot(m.t,m.q*180/pi);grid on;xlabel('Time (s)');ylabel('degrees');title('Six joint angles');
subplot(2,3,5);stairs(m.t,m.mode);grid on;xlabel('Time (s)');title('0 cruise | 1 slow | 2 stop | 3 arm | 4 scan');
subplot(2,3,6);bar(1000*[m.placeError;m.dockError]);set(gca,'XTickLabel',{'Place 1','Place 2','Dock'});ylabel('Error (mm)');grid on;title('Measured endpoint error');
print(f,fullfile(c.output,'mission_analysis.png'),'-dpng','-r150');
% Evaluate payload torque demands for exactly the same measured arm poses.
ii=unique(round(linspace(1,numel(m.t),120)));torque=zeros(numel(ii),3);
for k=1:numel(ii)
 for j=1:3,tt=mm_rne(m.q(ii(k),:)',zeros(6,1),zeros(6,1),c.arm,j-1);torque(k,j)=max(abs(tt));end
end
f2=figure('Name','Payload and singularity','Color','w','Position',[100 100 1100 420]);
subplot(1,2,1);plot(m.t(ii),torque,'LineWidth',1.2);grid on;legend('0 kg','1 kg','2 kg');xlabel('Time (s)');ylabel('Peak joint gravity torque (Nm)');title('Static payload sensitivity');
s=zeros(size(ii));for k=1:numel(ii),[~,J]=mm_fk(m.q(ii(k),:)',c.arm);s(k)=min(svd(J));end
subplot(1,2,2);plot(m.t(ii),s,'LineWidth',1.2);grid on;xlabel('Time (s)');ylabel('sigma min (mixed SI units)');title('Geometric Jacobian singularity indicator');
print(f2,fullfile(c.output,'payload_singularity.png'),'-dpng','-r150');
end
