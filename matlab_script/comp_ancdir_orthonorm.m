function [ res_all_ancdir, res_all_orthonorm ] = comp_ancdir_orthonorm()
%CALC_TRAJ_ERR Summary of this function goes here
%   Detailed explanation goes here

% errors = [0.01 0.1 1.0];
% ba_sizes = [5 10 20];
% errors = [1.0];
% ba_sizes = [5];
errors = [0.2 0.4 0.6 0.8 1.0];
ba_sizes = [5 10 20 40];
num_max_iter = [10 1000];
param_names = {'ancdir'; 'orthonorm'};

% res_all = cell(size(errors,1), size(ba_sizes,1), size(param_names,1));
res_all = zeros(size(errors,1), size(ba_sizes,1)*4, size(param_names,1));
res_avg = zeros(size(errors,1), size(ba_sizes,1)*2, size(param_names,1));
res_sum = zeros(size(errors,1), size(ba_sizes,1)*2, size(param_names,1));

trajgt_filename = 'gt_trajectory_wave.txt';
trajgt = importdata(trajgt_filename);

for err_i = 1:length(errors)
  
  for bs_i = 1:length(ba_sizes)
    
    for pn_i = 1:size(param_names,1)      
      
      param_name = char(param_names(pn_i));
      
      traj_filename = ...
        sprintf('trajectory_%s_err%.1f_basize%d_maxnumiter%d.txt', ...
        param_name, errors(err_i), ba_sizes(bs_i), 10);
      
      traj = importdata(traj_filename);
      num_traj = min(size(trajgt,1), size(traj,1));
      err_mat = trajgt(1:num_traj, 1:3) - traj(1:num_traj, 1:3);
%       err_mat = trajgt(1:num_traj, 1:6) - traj(1:num_traj, 1:6);
      err_sum = 0;
      
      result_filename = ...
        sprintf('ba_result_%s_err%.1f_basize%d_maxnumiter%d.txt', ...
        param_name, errors(err_i), ba_sizes(bs_i), 10);
      ba_result = importdata(result_filename);
      num_iter_avg = ba_result.data;
      total_time = str2double(ba_result.textdata(2,4));
      
      for i = 1:num_traj
        err_sum = err_sum + norm(err_mat(i,:));
      end
      
      err_avg = err_sum / num_traj;
      
      %       res_all{err_i, bs_i, pn_i} = ...
      %         sprintf('%.7f, %.3f, %.3f, %.3f', ...
      %         err_sum, err_avg, num_iter_avg, total_time);
      res_all(err_i, 4*bs_i-3, pn_i) = err_sum;
      res_all(err_i, 4*bs_i-2, pn_i) = err_avg;
      res_all(err_i, 4*bs_i-1, pn_i) = num_iter_avg;
      res_all(err_i, 4*bs_i, pn_i)   = total_time;
      res_avg(err_i, 2*bs_i-1, pn_i) = err_avg;
      res_avg(err_i, 2*bs_i, pn_i)   = num_iter_avg;
      res_sum(err_i, 2*bs_i-1, pn_i) = err_sum;
      res_sum(err_i, 2*bs_i, pn_i)   = num_iter_avg;
      
    end
    
  end
  
end

res_all_ancdir(:,:,1) = res_all(:,:,1);
res_all_orthonorm(:,:,1) = res_all(:,:,2);

for err_i = 1:length(errors)
  
  for bs_i = 1:length(ba_sizes)
    
    for pn_i = 1:size(param_names,1)      
      
      param_name = char(param_names(pn_i));
      
      traj_filename = ...
        sprintf('trajectory_%s_err%.1f_basize%d_maxnumiter%d.txt', ...
        param_name, errors(err_i), ba_sizes(bs_i), 1000);
      
      traj = importdata(traj_filename);
      num_traj = min(size(trajgt,1), size(traj,1));
      err_mat = trajgt(1:num_traj, 1:3) - traj(1:num_traj, 1:3);
%       err_mat = trajgt(1:num_traj, 1:6) - traj(1:num_traj, 1:6);
      err_sum = 0;
      
      result_filename = ...
        sprintf('ba_result_%s_err%.1f_basize%d_maxnumiter%d.txt', ...
        param_name, errors(err_i), ba_sizes(bs_i), 1000);
      ba_result = importdata(result_filename);
      num_iter_avg = ba_result.data;
      total_time = str2double(ba_result.textdata(2,4));
      
      for i = 1:num_traj
        err_sum = err_sum + norm(err_mat(i,:));
      end
      
      err_avg = err_sum / num_traj;
      
      %       res_all{err_i, bs_i, pn_i} = ...
      %         sprintf('%.7f, %.3f, %.3f, %.3f', ...
      %         err_sum, err_avg, num_iter_avg, total_time);
      res_all(err_i, 4*bs_i-3, pn_i) = err_sum;
      res_all(err_i, 4*bs_i-2, pn_i) = err_avg;
      res_all(err_i, 4*bs_i-1, pn_i) = num_iter_avg;
      res_all(err_i, 4*bs_i, pn_i)   = total_time;
      res_avg(err_i, 2*bs_i-1, pn_i) = err_avg;
      res_avg(err_i, 2*bs_i, pn_i)   = num_iter_avg;
      res_sum(err_i, 2*bs_i-1, pn_i) = err_sum;
      res_sum(err_i, 2*bs_i, pn_i)   = num_iter_avg;
      
    end
    
  end
  
end

res_all_ancdir(:,:,2) = res_all(:,:,1);
res_all_orthonorm(:,:,2) = res_all(:,:,2);


end

