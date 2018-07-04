% plot_trajectory.m
figure()
% hold on;
% plot(waypoints1(2,:),waypoints1(3,:),'b--');
% hold on;
plot(r1_pos.x.Data,r1_pos.y.Data,'r.');
% hold on;
% plot(waypoints2(2,:),waypoints2(3,:),'b--');
% hold on;
% plot(r1_pos1.x.Data,r1_pos1.y.Data,'b.');

set(gca,'XMinorGrid','on','YMinorGrid','on')
grid on;
hold on;
plot(0,0,'ro');
hold on;
plot(1,1,'go');
figure()
plot(r1_obs_cmd.Time,r1_obs_cmd.Data(:,1),'r.');
hold on;
plot(r1_pos.x.Time,r1_pos.x.Data);
figure()
plot(r1_obs_cmd.Time,r1_obs_cmd.Data(:,2),'r.');
hold on;
plot(r1_pos.y.Time,r1_pos.y.Data);