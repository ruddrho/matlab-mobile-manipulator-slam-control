function mission=run_project(animate)
% RUN_PROJECT  Complete mobile manipulation mission, Base MATLAB only.
%   run_project           numeric simulation + 3D replay + result plots
%   run_project(false)    numeric simulation + plots, no replay
if nargin<1,animate=true;end
here=fileparts(mfilename('fullpath'));addpath(fullfile(here,'src'));
c=mm_config();rng(c.seed);if ~exist(c.output,'dir'),mkdir(c.output);end
base=c.dock;q=c.arm.qHome;blocks=c.blocks;held=0;missionTime=0;
mission=struct();mission.t=[];mission.base=[];mission.q=[];mission.agv=[];
mission.clearance=[];mission.speed=[];mission.front=[];mission.pathError=[];mission.mode=[];
mission.scanProgress=[];mission.agvActive=[];mission.crossingId=[];mission.crossings={};
mission.blocks=[];mission.held=[];mission.stage={};mission.paths={};mission.events={};
agv.count=0;agv.enabled=true;agv.triggered=false;agv.active=false;agv.start=0;agv.origin=[0 0];agv.dir=[0 1];
placeError=zeros(2,1);
for b=1:2
 [base,L,agv]=mm_drive(base,c.pickBase,missionTime,c,agv);append(L,sprintf('Approach block %d',b));
 above=blocks(b,:)+[0 0 .30];
 [q,L]=mm_arm_move(q,above,base,missionTime,0,c,false);append(L,'Pre-grasp');
 [q,L]=mm_arm_move(q,blocks(b,:),base,missionTime,0,c,true);append(L,'Cartesian descend');
 F=mm_fk(q,c.arm);tcp=worldTCP(F,base);
 if norm(tcp-blocks(b,:))>.015,error('mm:Grasp','Grasp outside tolerance.');end
 held=b;mission.events{end+1}=sprintf('Grasp %d at %.2f s',b,missionTime);
 [q,L]=mm_arm_move(q,above,base,missionTime,c.payload,c,true);append(L,'Lift payload');
 [q,L]=mm_stow(q,base,missionTime,c.payload,c);append(L,'Stow arm');
 [base,L,agv]=mm_drive(base,c.goalBase,missionTime,c,agv);append(L,sprintf('Transport block %d',b));
 above=c.stack(b,:)+[0 0 .32];
 [q,L]=mm_arm_move(q,above,base,missionTime,c.payload,c,false);append(L,'Pre-place');
 [q,L]=mm_arm_move(q,c.stack(b,:),base,missionTime,c.payload,c,true);append(L,'Cartesian place');
 F=mm_fk(q,c.arm);blocks(b,:)=worldTCP(F,base);held=0;
 placeError(b)=norm(blocks(b,:)-c.stack(b,:));
 mission.events{end+1}=sprintf('Place %d at %.2f s; error %.6f m',b,missionTime,placeError(b));
 [q,L]=mm_arm_move(q,above,base,missionTime,0,c,true);append(L,'Retract');
 [q,L]=mm_stow(q,base,missionTime,0,c);append(L,'Stow arm');
end
[q,L]=mm_arm_move(q,[c.stack(2,1:2) .95],base,missionTime,0,c,false);append(L,'Wrist camera inspection');
[~,Jscan]=mm_fk(q,c.arm); %#ok<ASGLU>
L=mm_scan_hold(q,base,missionTime,c);append(L,'SCANNING RED + BLUE STACK');
mission.inspection=mm_inspect(q,base,blocks,c);
mission.events{end+1}=sprintf('Stack scanning completed at %.2f s; pass=%d',missionTime,mission.inspection.passed);
[q,L]=mm_stow(q,base,missionTime,0,c);append(L,'Return posture');
[base,L,agv]=mm_drive(base,c.dock,missionTime,c,agv);append(L,'Return and dock');
mission.agvCrossingCount=agv.count;
mission.routeLength=sum(sqrt(sum(diff(mission.base(:,1:2)).^2,2)));
mission.config=c;mission.finalBlocks=blocks;mission.placeError=placeError;
mission.dockError=norm(base(1:2)-c.dock(1:2));mission.dockYawError=abs(mm_wrap(base(3)-c.dock(3)));
mission.collisionSamples=sum(mission.clearance<0);mission.minClearance=min(mission.clearance);
mission.safetyStopSamples=sum(mission.mode==2);mission.completed=mission.inspection.passed && all(placeError<.015) && mission.dockError<.05 && mission.collisionSamples==0;
mission.wheelRates=[mission.speed(:,1)-c.track*mission.speed(:,2)/2 ...
 mission.speed(:,1)+c.track*mission.speed(:,2)/2]/c.wheelRadius;
save(fullfile(c.output,'mission.mat'),'mission','-v7');
mm_report(mission,c);mm_plots(mission,c);
if animate,mm_animate(mission,false);end
fprintf('Mission complete: %d | duration %.1f s | minimum clearance %.3f m\n',mission.completed,missionTime,mission.minClearance);

 function append(L,label)
  n=numel(L.t);if n==0,return;end
  names={'t','base','q','agv','clearance','speed','front','pathError','mode'};
  for ni=1:numel(names),key=names{ni};mission.(key)=[mission.(key);L.(key)];end
  if isfield(L,'path'),mission.paths{end+1}=L.path;end
  if isfield(L,'crossings'),mission.crossings{end+1}=L.crossings;end
  if ~isfield(L,'scanProgress'),L.scanProgress=zeros(n,1);end
  if ~isfield(L,'agvActive'),L.agvActive=zeros(n,1);end
  if ~isfield(L,'crossingId'),L.crossingId=zeros(n,1);end
  mission.scanProgress=[mission.scanProgress;L.scanProgress];
  mission.agvActive=[mission.agvActive;L.agvActive];
  mission.crossingId=[mission.crossingId;L.crossingId];
  for ii=1:n
   bb=blocks;
   if held>0,Ft=mm_fk(L.q(ii,:)',c.arm);bb(held,:)=worldTCP(Ft,L.base(ii,:));end
   mission.blocks(end+1,:)=reshape(bb',1,6);mission.held(end+1,1)=held;
   mission.stage{end+1,1}=label;
  end
  if held>0,blocks(held,:)=bb(held,:);end
  missionTime=L.t(end);
  fprintf('%7.2f s | %s\n',missionTime,label);
 end
end
function p=worldTCP(F,base)
B=mm_rot([0;0;1],base(3));p=(B*F(1:3,4))'+[base(1:2) 0];
end
