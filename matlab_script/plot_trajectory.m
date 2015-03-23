close all;

% traj_scavislam_itbt3f = importdata('traj_scavislam_itbt3f.txt');
% traj_scavislam_itbt3f(:,4) = ...
%   traj_scavislam_itbt3f(:,4) - traj_scavislam_itbt3f(1,4);
% traj_scavislam_itbt3f(:,5) = ...
%   traj_scavislam_itbt3f(:,5) - traj_scavislam_itbt3f(1,5);
% traj_scavislam_itbt3f(:,6) = ...
%   traj_scavislam_itbt3f(:,6) - traj_scavislam_itbt3f(1,6);
% 
% traj_scavislam_itbt3f(:,4:5) = -traj_scavislam_itbt3f(:,4:5);
% traj_scavislam_itbt3f_tmp = traj_scavislam_itbt3f;
% traj_scavislam_itbt3f(:,4) = traj_scavislam_itbt3f_tmp(:,6);
% traj_scavislam_itbt3f(:,5) = traj_scavislam_itbt3f_tmp(:,4);
% traj_scavislam_itbt3f(:,6) = traj_scavislam_itbt3f_tmp(:,5);
% 
% newx = traj_scavislam_itbt3f(10,4:6)';
% newy = [0 1 0; -1 0 0; 0 0 1]' * newx;
% newz = cross(newx, newy);
% newx = newx / norm(newx);
% newy = newy / norm(newy);
% newz = newz / norm(newz);
% 
% R = [newx newy newz];
% 
% traj_scavislam_itbt3f = traj_scavislam_itbt3f(:,4:6)';
% 
% traj_scavislam_itbt3f = R' * traj_scavislam_itbt3f;
% traj_scavislam_itbt3f = traj_scavislam_itbt3f';
% 
% figure(1);
% plot3(traj_scavislam_itbt3f(:,1), ...
%   traj_scavislam_itbt3f(:,2), ...
%   traj_scavislam_itbt3f(:,3), ...
%   'bs', 'LineWidth', 2, ...
%   'MarkerEdgeColor', 'r', ...
%   'MarkerFaceColor', 'r', ...  
%   'MarkerSize', 4);
% axis equal;
% % xlabel('x');
% % ylabel('y');
% % zlabel('z');
% axis([-7 22 -3 17 -3 2]);
% view([0 0 1]);
% grid on;

traj_slslam_itbt3f = importdata('traj_slslam_itbt3f_basize10_wolc.txt');
newx = traj_slslam_itbt3f(10,2:4)';
newy = [0 1 0; -1 0 0; 0 0 1]' * newx;
newz = cross(newx, newy);
newx = newx / norm(newx);
newy = newy / norm(newy);
newz = newz / norm(newz);
R = [newx newy newz];
traj_slslam_itbt3f = traj_slslam_itbt3f(:,2:4)';
traj_slslam_itbt3f = R' * traj_slslam_itbt3f;
traj_slslam_itbt3f = traj_slslam_itbt3f';
figure(2);
plot3(traj_slslam_itbt3f(:,1), ...
  traj_slslam_itbt3f(:,2), ...
  traj_slslam_itbt3f(:,3), ...
  'bs', 'LineWidth', 2, ...
  'MarkerEdgeColor', 'b', ...
  'MarkerFaceColor', 'b', ...
  'MarkerSize', 4);
axis equal;
axis([-7 22 -3 17 -3 2]);
view([0 0 1]);
grid on;

% traj_scavislam_myungdong = importdata('traj_scavislam_myungdong.txt');
% traj_scavislam_myungdong(:,4) = ...
%   traj_scavislam_myungdong(:,4) - traj_scavislam_myungdong(1,4);
% traj_scavislam_myungdong(:,5) = ...
%   traj_scavislam_myungdong(:,5) - traj_scavislam_myungdong(1,5);
% traj_scavislam_myungdong(:,6) = ...
%   traj_scavislam_myungdong(:,6) - traj_scavislam_myungdong(1,6);
% 
% traj_scavislam_myungdong(:,4:5) = -traj_scavislam_myungdong(:,4:5);
% traj_scavislam_myungdong_tmp = traj_scavislam_myungdong;
% traj_scavislam_myungdong(:,4) = traj_scavislam_myungdong_tmp(:,6);
% traj_scavislam_myungdong(:,5) = traj_scavislam_myungdong_tmp(:,4);
% traj_scavislam_myungdong(:,6) = traj_scavislam_myungdong_tmp(:,5);
% 
% newx = traj_scavislam_myungdong(2,4:6)';
% newy = [0 1 0; -1 0 0; 0 0 1]' * newx;
% newz = cross(newx, newy);
% newx = newx / norm(newx);
% newy = newy / norm(newy);
% newz = newz / norm(newz);
% 
% R = [newx newy newz];
% 
% traj_scavislam_myungdong = traj_scavislam_myungdong(:,4:6)';
% traj_scavislam_myungdong = R' * traj_scavislam_myungdong;
% traj_scavislam_myungdong = traj_scavislam_myungdong';
% 
% figure(3);
% plot3(traj_scavislam_myungdong(:,1), ...
%   traj_scavislam_myungdong(:,2), ...
%   traj_scavislam_myungdong(:,3), ...
%   'bs', 'LineWidth', 2, ...
%   'MarkerEdgeColor', 'r', ...
%   'MarkerFaceColor', 'r', ...  
%   'MarkerSize', 4);
% axis equal;
% % xlabel('x');
% % ylabel('y');
% % zlabel('z');
% axis([-15 65 -40 10 -10 5]);
% grid on;

traj_slslam_myungdong = importdata('traj_slslam_myungdong_basize10_wolc.txt');
newx = traj_slslam_myungdong(50,2:4)';
newy = [0 1 0; -1 0 0; 0 0 1]' * newx;
newz = cross(newx, newy);
newx = newx / norm(newx);
newy = newy / norm(newy);
newz = newz / norm(newz);
R = [newx newy newz];
traj_slslam_myungdong = traj_slslam_myungdong(:,2:4)';
traj_slslam_myungdong = R' * traj_slslam_myungdong;
traj_slslam_myungdong = traj_slslam_myungdong';
figure(4);
plot3(traj_slslam_myungdong(:,1), traj_slslam_myungdong(:,2), ...
  traj_slslam_myungdong(:,3), ...
  'bs', 'LineWidth', 2, ...
  'MarkerEdgeColor', 'b', ...
  'MarkerFaceColor', 'b', ...
  'MarkerSize', 4);
axis equal;
axis([-15 65 -40 10 -10 5]);
grid on;

% traj_scavislam_olympic4f = importdata('traj_scavislam_olympic4f.txt');
% traj_scavislam_olympic4f(:,4) = ...
%   traj_scavislam_olympic4f(:,4) - traj_scavislam_olympic4f(1,4);
% traj_scavislam_olympic4f(:,5) = ...
%   traj_scavislam_olympic4f(:,5) - traj_scavislam_olympic4f(1,5);
% traj_scavislam_olympic4f(:,6) = ...
%   traj_scavislam_olympic4f(:,6) - traj_scavislam_olympic4f(1,6);
% 
% traj_scavislam_olympic4f(:,4:5) = -traj_scavislam_olympic4f(:,4:5);
% traj_scavislam_olympic4f_tmp = traj_scavislam_olympic4f;
% traj_scavislam_olympic4f(:,4) = traj_scavislam_olympic4f_tmp(:,6);
% traj_scavislam_olympic4f(:,5) = traj_scavislam_olympic4f_tmp(:,4);
% traj_scavislam_olympic4f(:,6) = traj_scavislam_olympic4f_tmp(:,5);
% 
% newx = traj_scavislam_olympic4f(50,4:6)';
% newy = [0 1 0; -1 0 0; 0 0 1]' * newx;
% newz = cross(newx, newy);
% newx = newx / norm(newx);
% newy = newy / norm(newy);
% newz = newz / norm(newz);
% 
% R = [newx newy newz];
% 
% traj_scavislam_olympic4f = traj_scavislam_olympic4f(:,4:6)';
% 
% traj_scavislam_olympic4f = R' * traj_scavislam_olympic4f;
% traj_scavislam_olympic4f = traj_scavislam_olympic4f';
% 
% figure(5);
% plot3(traj_scavislam_olympic4f(:,1), ...
%   traj_scavislam_olympic4f(:,2), ...
%   traj_scavislam_olympic4f(:,3), ...
%   'bs', 'LineWidth', 2, ...
%   'MarkerEdgeColor', 'r', ...
%   'MarkerFaceColor', 'r', ...  
%   'MarkerSize', 4);
% axis equal;
% % xlabel('x');
% % ylabel('y');
% % zlabel('z');
% axis([-15 65 -65 15 -10 10]);
% view([0 0 1]);
% grid on;

traj_slslam_olympic4f= importdata('traj_slslam_olympic4f_basize10_wolc.txt');
newx = traj_slslam_olympic4f(50,2:4)';
newy = [0 1 0; -1 0 0; 0 0 1]' * newx;
newz = cross(newx, newy);
newx = newx / norm(newx);
newy = newy / norm(newy);
newz = newz / norm(newz);
R = [newx newy newz];
traj_slslam_olympic4f = traj_slslam_olympic4f(:,2:4)';
traj_slslam_olympic4f = R' * traj_slslam_olympic4f;
traj_slslam_olympic4f = traj_slslam_olympic4f';

figure(6);
plot3(traj_slslam_olympic4f(:,1), ...
  traj_slslam_olympic4f(:,2), ...
  traj_slslam_olympic4f(:,3), ...
  'bs', 'LineWidth', 2, ...
  'MarkerEdgeColor', 'b', ...
  'MarkerFaceColor', 'b', ...
  'MarkerSize', 4);
axis equal;
% xlabel('x');
% ylabel('y');
% zlabel('z');
axis([-15 65 -65 15 -10 10]);
% view([0 -1 0]);
view([0 0 1]);
grid on;
