function s=mm_slam_init(c,initialPose)
% Empty occupancy map; only world extent and initial pose are known.
s.res=.10;s.x=c.world(1):s.res:c.world(2);s.y=c.world(3):s.res:c.world(4);
s.L=zeros(numel(s.y),numel(s.x));s.pose=initialPose;s.odom=initialPose;
s.trail=initialPose(1:2);s.hits=zeros(0,2);s.count=0;s.period=.20;
end
