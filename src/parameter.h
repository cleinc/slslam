// SLSLAM: Stereo Line-based SLAM
// This file is part of SLSLAM
//
// Copyright (C) 2015 Guoxuan Zhang, Jin Han Lee, Jongwoo Lim, Il Hong Suh
// 
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

#ifndef PARAMETER_H_
#define PARAMETER_H_

namespace {

  const int max_feat_num = 5;
  const int max_trials = 1000;

#ifdef IT3F
  const string img_dir = "../data/it3f/left_rect";
  const string obs_dir = "../data/it3f/line_tracking_result";
#endif

#ifdef OLYMPIC4F
  const string img_dir = "../data/olympic4f/left_rect";
  const string obs_dir = "../data/olympic4f/line_tracking_result";
#endif

#ifdef MYUNGDONG
  const string img_dir = "../data/myung-dong/left_rect";
  const string obs_dir = "../data/myung-dong/line_tracking_result";
#endif

  const int image_width = 640;
  const int image_height = 480;

  const double baseline = 0.12;
  const double focal_length = 406.05;
  const double cx1 = 327.783;
  const double cy1 = 237.172;

  const double fx1 = focal_length;
  const double fy1 = focal_length;
  const double prob_free_outliers = 0.999;

  const double inverse_depth = 0.1;
  const double error_thr = 5 / focal_length; // 1.5, 2.236,
  // const double kf_rot_thr = 3 * 3.141592654 / 180;
  // const double kf_tr_thr = 0.2;
  const double kf_rot_thr = 15 * 3.141592654 / 180;
  const double kf_tr_thr = 0.75;
  const double line_vn_angle_thr = 3 * 3.141592654 / 180;
  const double extension_length = 5.0;
}

#endif /* PARAMETER_H_ */
