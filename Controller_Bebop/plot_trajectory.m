% plot_trajectory.m
figure()
% hold on;
plot(waypoints1(2,:),waypoints1(3,:),'b--');
hold on;
plot(r1_pos.x.Data,r1_pos.y.Data,'r.');
% hold on;
% plot(waypoints2(2,:),waypoints2(3,:),'b--');
% hold on;
% plot(r1_pos1.x.Data,r1_pos1.y.Data,'b.');

set(gca,'XMinorGrid','on','YMinorGrid','on')
grid on;