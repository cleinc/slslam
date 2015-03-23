h=house;
h([1 4], :) = h([1 4], :) - 2.25;
h([2 5], :) = h([2 5], :) + 2.75;
h([3 6], :) = h([3 6], :);
figure(1);
trajgt_filename = 'gt_trajectory_wave.txt';
trajgt = importdata(trajgt_filename);

plot3(trajgt(:,3), trajgt(:,1), trajgt(:,2)+1.5,'LineWidth',2);
hold;
plot3(h([1 4],:),h([2 5],:),h([3 6],:),'LineWidth',2);
grid on;
axis equal;
