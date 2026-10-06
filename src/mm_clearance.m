function d=mm_clearance(pose,agv,c)
d=min(hypot(c.obstacles(:,1)-pose(1),c.obstacles(:,2)-pose(2))-c.obstacles(:,3)-c.baseRadius);
d=min(d,mm_station_clearance(pose(1:2),pose(1:2),c));
d=min(d,norm(pose(1:2)-agv(1:2))-c.agvRadius-c.baseRadius);
d=min([d pose(1)-c.world(1)-c.baseRadius c.world(2)-pose(1)-c.baseRadius ...
 pose(2)-c.world(3)-c.baseRadius c.world(4)-pose(2)-c.baseRadius]);
end
