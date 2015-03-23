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

#ifndef SLAM_H_
#define SLAM_H_

#include "all.h"
#include "parameter.h"
#include "cplot.h"
#include "lba_problem.h"
#include "po_problem.h"
#include "stopwatch.h"
#include "hrandom.h"

// #include "place_recognition.h"

using namespace hcv;

typedef map<int,int> mii;
typedef pair<int,int> pii;

typedef struct _obs_t {
  _obs_t() {}
  _obs_t( int id, Vector8d obs ) : id( id ), obs( obs ) {}

  int id;
  Vector8d obs;
} obs_t;   // id: feature_id or kf_id

typedef struct {
  pose_t T;
  set<int> member_lms;
  set<int> neighbor_kfs;
} keyframe_t;

typedef struct _edge_t {
  _edge_t() {}
  _edge_t( pose_t T ) : T( T ), C( T ) {}

  _edge_t inverse() {
    return _edge_t( pose_t( T.R.inverse(), - T.R.inverse() * T.t ) );
  }

  pose_t T;
  pose_t C;
} edge_t;

typedef struct {
  Vector6d line;
  Vector2d tt;
  Vector3d pvn;
  bool twice_observed;
  bool ba_updated;
  bool currently_visible;
  int init_kfid;
  vector<obs_t> obs_vec;
} landmark_t;

// typedef struct SEGMENT segment_t;

typedef map<int, keyframe_t*> kf_map;
typedef map<int, landmark_t*> lm_map;
typedef map<int, Vector8d> ob_map;
typedef map<pii, edge_t> edge_map;
// metric embedding
typedef multimap<double, int> me_map;

//#define DRAW_ALL_VIEW
#define SAVE_SLAM_RESULTS

class SLAM {

public:
  SLAM();
  virtual ~SLAM();

  void start_slam_cycle( int id );
  void add_new_keyframe( bool add_edge );
  void pose_optimization();
  void local_bundle_adjustment();
  void draw();
  void end_slam_cycle();
  void post_processing();
  void time_tag( string str );
  int check_input_data();
  bool loop_closure();
  bool check_keyframe_motion();
  bool grab_new_frame( int id );
  bool place_recognized();
  bool consistency_broken();
  void save_trajectory();
  void save_landmark();

  double m_sum_final_cost;
  double m_sum_init_cost;
  int m_sum_num_iteration;
  double m_total_time;
  int m_num_keyframes() { return kfs.size();}

protected:

  void add_kf_member_lms( keyframe_t * kf );
  void add_lms();
  void motion_only_ba( ob_map obs0, ob_map obs1, vector<int> comm_feat,
      map<int, Vector6d> lines, pose_t & T, vector<int> & inliers );
  void ransac_motion( vector<Vector8d> obs0, vector<Vector8d> obs1,
      int max_feat_num, int & best_score, int & trial_cnt,
      vector<int> comm_feat, map<int, Vector6d> lines,
      pose_t & best_T, vector<int> & best_inliers );
  void extend_end_points();
  void bundle_adjustment();
  void delete_lms();
  void insert_curr_obs( int feature_id, Vector8d obs );
  void metric_embedding( int id, me_map & mes );
  int  vo_angle_axis_approx( pose_t T[10], vector<Vector8d> obs0,
      vector<Vector8d>  obs1, int nfeat, float baseline );
  bool  pose_estimation( ob_map obs1, ob_map obs2, pose_t & motion );
  float reprojection_error( Vector8d ft, pose_t T, Vector6d line );
  Vector6d initialize_lm( Vector8d obs, double baseline );
  Vector6d line_transform( landmark_t * lm, pose_t T2 );

  inline void set_frame_id( int id ) { frame_id = id; }

  int lc_kf_id;
  int lc_cnt;
  int frame_id;

  HRandom rand;
  CPlot plot;
  StopWatch stop_watch;
  // place_recognition pr;

  pose_t curr_pose;

  set<int> final_inliers;
  set<int> prev_ba_kfs, curr_ba_kfs;
  set<pii> edge_set;
  // vector<segment_t> pr_observations;
  mii ba_kfs;
  mii match_result;
  mii match_lookup;

  ob_map prev_kf_obs, curr_obs;
  kf_map kfs;
  lm_map lms;
  edge_map edges;

};

#endif /* SLAM_H_ */
