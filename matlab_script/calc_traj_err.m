function [ res_avg, res_sum ] = calc_traj_err()
%CALC_TRAJ_ERR Summary of this function goes here
%   Detailed explanation goes here

errors = [0.2 0.4 0.6 0.8 1.0];
ba_sizes = [5 10 20 40];
param_names = {'ancdir'; 'orthonorm'; 'aid'};

res_avg = zeros(size(errors,1), size(ba_sizes,1)*2, size(param_names,1));
res_sum = zeros(size(errors,1), size(ba_sizes,1)*2, size(param_names,1));

trajgt_filename = 'gt_trajectory_wave.txt';
trajgt = importdata(trajgt_filename);

for err_i = 1:length(errors)
  
  for bs_i = 1:length(ba_sizes)
    
    for pn_i = 1:size(param_names,1)      
      
      param_name = char(param_names(pn_i));
      
      traj_filename = sprintf('trajectory_%s_err%.1f_basize%d.txt', ...
        param_name, errors(err_i), ba_sizes(bs_i));
      
      traj = importdata(traj_filename);
      num_traj = min(size(trajgt,1), size(traj,1));
      err_mat = trajgt(1:num_traj, 1:3) - traj(1:num_traj, 1:3);
      err_sum = 0;
      
      result_filename = sprintf('ba_result_%s_err%.1f_basize%d.txt', ...
        param_name, errors(err_i), ba_sizes(bs_i));
      ba_result = importdata(result_filename);
      num_iter_avg = ba_result.data;
      
      for i = 1:num_traj
        err_sum = err_sum + norm(err_mat(i,:));
      end
      
      err_avg = err_sum / num_traj;
      
      res_avg(err_i, 2*bs_i-1, pn_i) = err_avg;
      res_avg(err_i, 2*bs_i, pn_i) = num_iter_avg;
      res_sum(err_i, 2*bs_i-1, pn_i) = err_sum;
      res_sum(err_i, 2*bs_i, pn_i) = num_iter_avg;
      
    end
    
  end
  
end

end

