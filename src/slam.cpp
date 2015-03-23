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

#include "slam.h"
#include "gc.h"

DECLARE_double(obs_err_stddev);
DECLARE_int32(ba_window_size);
DECLARE_int32(max_num_iter);
DECLARE_int32(rseed);

//////////////////////////////////////////////////////////////////////////////

SLAM::SLAM() {
  lc_cnt = 0;
  curr_pose = pose_t();

  rand.init( FLAGS_rseed );
  stop_watch.start();

  m_sum_init_cost = 0.0;
  m_sum_final_cost = 0.0;
  m_sum_num_iteration = 0;
}

//////////////////////////////////////////////////////////////////////////////

SLAM::~SLAM() {
  stop_watch.stop();
}

//////////////////////////////////////////////////////////////////////////////

void SLAM::start_slam_cycle( int id ) {
  set_frame_id( id );
  cout << frame_id << ":";

  curr_obs.clear();
  ba_kfs.clear();
  match_result.clear();
  // pr_observations.clear();
}

//////////////////////////////////////////////////////////////////////////////

bool SLAM::grab_new_frame( int id ) {
  if ( id == 0 )
    return true;

  set_frame_id( id );
  for(lm_map::iterator mit = lms.begin(); mit != lms.end(); ++mit) {
    mit->second->currently_visible = false;
  }

  char line[256];
  char file_name[256];

  sprintf( file_name, "%s/%04d.txt", obs_dir.c_str(), id );

  ifstream fp;
  fp.open( file_name, ios::in );

  if ( fp.fail() ) {
    return false;
  } else {
    int feature_id;
    Vector8d obs;

    while ( fp.getline( line, 256 ) ) {
      feature_id = atoi( strtok( line, " " ) );
      obs(0)     = atof( strtok( NULL, " " ) );
      obs(1)     = atof( strtok( NULL, " " ) );
      obs(2)     = atof( strtok( NULL, " " ) );
      obs(3)     = atof( strtok( NULL, " " ) );
      obs(4)     = atof( strtok( NULL, " " ) );
      obs(5)     = atof( strtok( NULL, " " ) );
      obs(6)     = atof( strtok( NULL, " " ) );
      obs(7)     = atof( strtok( NULL, " " ) );
      strtok( NULL, " \n" );
      insert_curr_obs( feature_id, obs );
      lm_map::iterator lmit = lms.find(feature_id);
      if(lmit != lms.end()) {
        lmit->second->currently_visible = true;
      }
      else {
        // printf("Newly observed landmark!\n");
      }
    }
  }
  fp.close();
  return true;
}

//////////////////////////////////////////////////////////////////////////////

void SLAM::insert_curr_obs( int feature_id, Vector8d obs ) {
//  segment_t seg;
//  seg.Label = feature_id;
//  seg.x1 = obs(0);
//  seg.y1 = obs(1);
//  seg.x2 = obs(2);
//  seg.y2 = obs(3);
//  pr_observations.push_back( seg );

  obs(0) = obs(0) / fx1 - cx1 / fx1;
  obs(1) = obs(1) / fy1 - cy1 / fy1;
  obs(2) = obs(2) / fx1 - cx1 / fx1;
  obs(3) = obs(3) / fy1 - cy1 / fy1;
  obs(4) = obs(4) / fx1 - cx1 / fx1;
  obs(5) = obs(5) / fy1 - cy1 / fy1;
  obs(6) = obs(6) / fx1 - cx1 / fx1;
  obs(7) = obs(7) / fy1 - cy1 / fy1;

  mii::iterator it = match_lookup.find( feature_id );
  if ( it != match_lookup.end() )
    feature_id = it->second;

  curr_obs.insert( make_pair( feature_id, obs ) );
}

//////////////////////////////////////////////////////////////////////////////

int SLAM::check_input_data() {
  if ( curr_obs.size() == 0 )
    return 1;

  if ( prev_kf_obs.size() == 0 )
    return 2;

  return 0;
}

//////////////////////////////////////////////////////////////////////////////

void SLAM::add_kf_member_lms( keyframe_t * kf ) {
  for ( set<int>::iterator it = final_inliers.begin();
      it != final_inliers.end(); ++it ) {
    kf->member_lms.insert( * it );
    kfs.rbegin()->second->member_lms.insert( * it );
  }
}

//////////////////////////////////////////////////////////////////////////////

void SLAM::add_lms() {

  int kfid = kfs.size() > 0 ? kfs.rbegin()->first : -1;

  for ( ob_map::iterator obit = curr_obs.begin(); obit != curr_obs.end();
      ++obit ) {

    lm_map::iterator lmit = lms.find( obit->first );
    if ( lmit == lms.end() ) {
      landmark_t* lm = new landmark_t;
      lm->line = initialize_lm( obit->second, baseline );
      lm->twice_observed = false;
      lm->ba_updated = false;
      lm->currently_visible= false;
      lm->tt = Vector2d( 0, 0 );
      lm->obs_vec.push_back( obs_t( kfid + 1, obit->second ) );
      lm->init_kfid = kfid + 1;
      Vector3d dv = lm->line.tail(3);
      lm->pvn = gc_vec3_normalize( dv );
      lms.insert( make_pair( obit->first, lm ) );
    } else {
      lmit->second->obs_vec.push_back( obs_t( kfid + 1, obit->second ) );
    }
  }

}

//////////////////////////////////////////////////////////////////////////////

Vector6d SLAM::initialize_lm( Vector8d obs, double baseline ) {
  Vector6d line;

  Vector3d p1( obs(0), obs(1), 1 );
  Vector3d p2( obs(2), obs(3), 1 );
  Vector3d p3( obs(4) + baseline, obs(5), 1 );
  Vector3d p4( obs(6) + baseline, obs(7), 1 );

  Vector4d pi1 = gc_ppp_pi( p1, p2, Vector3d( 0, 0, 0 ) );
  Vector4d pi2 = gc_ppp_pi( p3, p4, Vector3d( baseline, 0, 0 ) );

  Vector6d plk = gc_pipi_plk( pi1, pi2 );
  Vector3d n = plk.head(3);
  Vector3d v = plk.tail(3);
  Vector3d cp = gc_plucker_origin( n, v );

  double cpn = cp.norm();
  if ( cpn < 0.1 || cpn > 10.0) {
    Vector3d cpu = cp / cpn;
    // cout << "cpn = " << cpn << endl;
    cp = cpu / inverse_depth;
  }
  if ( cp(2) < 0 )
    cp = - cp;

  line.head(3) = cp;
  line.tail(3) = v;

  return line;
}

//////////////////////////////////////////////////////////////////////////////

bool SLAM::check_keyframe_motion() {
  me_map mes;
  metric_embedding( kfs.rbegin()->first, mes );

  pose_t best_motion = pose_t();

  if ( !pose_estimation( prev_kf_obs, curr_obs, best_motion ) )
    return false;

  if ( gc_Rodriguez( best_motion.R ).norm() < kf_rot_thr &&
      best_motion.t.norm() < kf_tr_thr )
    return false;

  curr_pose = best_motion;
  cout << "\t" << gc_Rt_to_wt( gc_T_inv( curr_pose ) ).transpose() << endl;

  return true;
}

//////////////////////////////////////////////////////////////////////////////

bool SLAM::pose_estimation( ob_map obs0, ob_map obs1, pose_t & motion ) {
  stop_watch.proc_1_tick();

  vector<int> comm_feat;
  map<int, Vector6d> lines;
  //  vector<Vector6d> line_vec;
  vector<Vector8d> obs0_vec, obs1_vec;

  ob_map::iterator it0 = obs0.begin();
  ob_map::iterator it1 = obs1.begin();

  while ( it0 != obs0.end() && it1 != obs1.end() ) {
    if ( it0->first == it1->first ) {
      comm_feat.push_back( it0->first );
      obs0_vec.push_back( obs0[ it0->first ] );
      obs1_vec.push_back( obs1[ it1->first ] );

      landmark_t * lm = lms[ it0->first ];
      lines.insert( make_pair( it0->first,
            gc_line_from_pose( lm->line, kfs[ lm->init_kfid ]->T ) ) );

      it0++;
      it1++;
    } else if ( it0->first < it1->first ) {
      it0++;
    } else {
      it1++;
    }
  }

  size_t comm_size = comm_feat.size();
  if ( comm_size < 5 ) {
    cout << "\tToo few features: " << comm_size << "\n";
    return false;
  }

  //------------------------------------------------------------------------//
  // RANSAC

  int best_score = -1;
  int trial_cnt = 0;

  vector<int> inliers;

  ransac_motion( obs0_vec, obs1_vec, max_feat_num, best_score, trial_cnt,
      comm_feat, lines, motion, inliers );
  //  pose_t tttttt = motion;


  cout << "\t" << "Trial: " << trial_cnt << " ";
  cout << "Feature Num: "<< comm_size << "-" << best_score;
  if ( best_score < max_feat_num ) {
    cout << "\t---" << endl;
    return false;
  }

  //------------------------------------------------------------------------//
  // Opimize pose

  motion_only_ba( obs0, obs1, comm_feat, lines, motion, inliers );

  final_inliers.clear();

  for ( size_t j = 0; j < comm_size; ++j ) {
    float error = reprojection_error( obs1_vec[ j ],
        motion, lines[ comm_feat[j] ] );
    if ( error < error_thr )
      final_inliers.insert( comm_feat[j] );
  }

  cout << "-" << final_inliers.size() << "\n";

  stop_watch.proc_1_tock();

  return true;
}

//////////////////////////////////////////////////////////////////////////////

void SLAM::ransac_motion( vector<Vector8d> obs0, vector<Vector8d> obs1,
    int max_feat_num, int & best_score, int & trial_cnt,
    vector<int> comm_feat, map<int, Vector6d> lines,
    pose_t & best_T, vector<int> & best_inliers ) {

  //  static HRandom _rand;

  size_t comm_size = comm_feat.size();
  int s = max_feat_num;
  int ransac_trial = comm_size;

  //  feat3d_t* ft0 = new feat3d_t[comm_size];
  //  feat3d_t* ft1 = new feat3d_t[comm_size];
  //
  //  for ( size_t j = 0; j < comm_size; ++j ) {
  //    Vector8d prev_data = obs0[ j ];
  //    ft0[j].id = 1;
  //    ft0[j].p0[0] = prev_data(0);
  //    ft0[j].p0[1] = prev_data(1);
  //    ft0[j].q0[0] = prev_data(2);
  //    ft0[j].q0[1] = prev_data(3);
  //    ft0[j].p1[0] = prev_data(4);
  //    ft0[j].p1[1] = prev_data(5);
  //    ft0[j].q1[0] = prev_data(6);
  //    ft0[j].q1[1] = prev_data(7);
  //
  //    Vector8d curr_data = obs1[ j ];
  //    ft1[j].id = 1;
  //    ft1[j].p0[0] = curr_data(0);
  //    ft1[j].p0[1] = curr_data(1);
  //    ft1[j].q0[0] = curr_data(2);
  //    ft1[j].q0[1] = curr_data(3);
  //    ft1[j].p1[0] = curr_data(4);
  //    ft1[j].p1[1] = curr_data(5);
  //    ft1[j].q1[0] = curr_data(6);
  //    ft1[j].q1[1] = curr_data(7);
  //  }

  //------------------------------------------------------------------------//

  for ( ; trial_cnt < ransac_trial && trial_cnt <= max_trials; ++trial_cnt ){

    vector<int> sample;
    rand.rand_sample( sample, comm_size, s );

    pose_t motion[10];

#if 0
    motion_t mot[10]
      const feat3d_t* cft0[s];
    const feat3d_t* cft1[s];
    for ( int j = 0; j < s; ++ j ) {
      cft0[j] = & ft0[sample[j]];
      cft1[j] = & ft1[sample[j]];
    }

    int num_sol = _compute_motion_tfq( mot, cft0, cft1, s, -baseline );

    for ( int j = 0; j < num_sol; ++j )
      for ( int k = 0; k < 6; ++k )
        motion[j][k] = mot[j][k];
#else
    vector<Vector8d> obs0_sample, obs1_sample;
    for ( int j = 0; j < s; ++ j ) {
      obs0_sample.push_back( obs0[ sample[j] ] );
      obs1_sample.push_back( obs1[ sample[j] ] );
    }

    int num_sol = vo_angle_axis_approx(
        motion, obs0_sample, obs1_sample, s, -baseline );
#endif

    if ( num_sol == 0 ) continue;

    vector<int> inliers;
    inliers.reserve(comm_size);

    for ( int j = 0; j < num_sol; ++j ) {
      if ( motion[j].t.norm() > 1 )
        continue;

      int score = 0;
      inliers.clear();
      for ( size_t k = 0; k < comm_size; ++k ) {
        float error = reprojection_error( obs1[ k ],
            motion[j], lines[ comm_feat[k] ] );
        if ( error < error_thr ) {
          score++;
          inliers.push_back( comm_feat[k] );
        }
      }

      if ( score > best_score ) {
        best_score = score;
        best_T = motion[j];
        best_inliers.swap( inliers );
        double prob_s_outliers = 1 - pow( score / (double) comm_size, s );
        ransac_trial = ( int ) ( log( 1 - prob_free_outliers ) /
            log( min( 1 - 1e-6, max( 1e-6, prob_s_outliers ))));
      }
    } // for ( int j = 0; j < num_sol; ++j )

  } //

  //  delete[] ft0;
  //  delete[] ft1;
}

//////////////////////////////////////////////////////////////////////////////

int SLAM::vo_angle_axis_approx( pose_t T[10], vector<Vector8d> obs0,
    vector<Vector8d>  obs1, int nfeat, float baseline ) {
  MatrixXd K( 2 * nfeat, 4 );

  for ( int i = 0; i < nfeat; ++i ) {
    Vector3d p1, p2, l1, l2, l3, l4, tl, lx, ly;

    p1 << obs0[i][0], obs0[i][1], 1;
    p2 << obs0[i][2], obs0[i][3], 1;
    l1 = p1.cross(p2);

    p1 << obs0[i][4], obs0[i][5], 1;
    p2 << obs0[i][6], obs0[i][7], 1;
    l2 = p1.cross(p2);

    p1 << obs1[i][0], obs1[i][1], 1;
    p2 << obs1[i][2], obs1[i][3], 1;
    l3 = p1.cross(p2);

    p1 << obs1[i][4], obs1[i][5], 1;
    p2 << obs1[i][6], obs1[i][7], 1;
    l4 = p1.cross(p2);

    lx = l1.cross(l2);
    double lxn = lx.norm();
    if ( lxn == 0 ) return 0;
    lx = lx / lxn;

    for ( int j = 0; j < 2; ++j ) {
      if ( j == 0 )
        tl = l3;
      else
        tl = l4;

      double tln = tl.norm();
      if ( tln == 0 ) return 0;
      ly = tl / tln;
      K.row( 2 * i + j ) <<
        lx(2) * ly(1) - lx(1) * ly(2),
        lx(0) * ly(2) - lx(2) * ly(0),
        lx(1) * ly(0) - lx(0) * ly(1),
        lx(0) * ly(0) + lx(1) * ly(1) + lx(2) * ly(2);
    }
  }

  MatrixXd A = K.topLeftCorner( 2 * nfeat, 3 );
  VectorXd b = - K.col(3);
  MatrixXd At = A.transpose();
  MatrixXd AtAi = ( At * A ).inverse();
  Vector3d w = - AtAi * At * b;
  Matrix3d R = gc_Rodriguez( w );

  Vector3d a4( baseline, 0, 0 );
  MatrixXd M( 6 * nfeat, 4 );

  //------------------------------------------------------------------------//

  for ( int i = 0; i < nfeat; ++i ) {
    Vector3d p1, p2, l1, l2, l3, lx, ly;

    p1 << obs0[i][0], obs0[i][1], 1;
    p2 << obs0[i][2], obs0[i][3], 1;
    l1 = p1.cross(p2);
    double l1n = l1.norm();
    if ( l1n == 0 ) return 0;
    l1 = l1 / l1n;

    p1 << obs0[i][4], obs0[i][5], 1;
    p2 << obs0[i][6], obs0[i][7], 1;
    l2 = p1.cross(p2);
    double l2n = l2.norm();
    if ( l2n == 0 ) return 0;
    l2 = l2 / l2n;

    lx = l1.cross(l2);
    double lxn = lx.norm();
    if ( lxn == 0 ) return 0;
    lx = lx / lxn;

    double c1, c2, c3;

    for ( int j = 0; j < 2; ++j ) {
      Matrix3d a4b1 = a4 * R.col(0).transpose();
      Matrix3d a4b2 = a4 * R.col(1).transpose();
      Matrix3d a4b3 = a4 * R.col(2).transpose();

      if ( j == 0 ) {
        p1 << obs1[i][0], obs1[i][1], 1;
        p2 << obs1[i][2], obs1[i][3], 1;
        l3 = p1.cross(p2);
        double l3n = l3.norm();
        if ( l3n == 0 ) return 0;
        l3 = l3 / l3n;

        c1 = - l2.transpose() * a4b1 * l3;
        c2 = - l2.transpose() * a4b2 * l3;
        c3 = - l2.transpose() * a4b3 * l3;
      } else {
        p1 << obs1[i][4], obs1[i][5], 1;
        p2 << obs1[i][6], obs1[i][7], 1;
        l3 = p1.cross(p2);
        double l3n = l3.norm();
        if ( l3n == 0 ) return 0;
        l3 = l3 / l3n;

        c1 = - l2.transpose() * a4b1 * l3 + l2(0) * baseline * l3(0);  // +
        c2 = - l2.transpose() * a4b2 * l3 + l2(1) * baseline * l3(0);;
        c3 = - l2.transpose() * a4b3 * l3 + l2(2) * baseline * l3(0);;
      }

      M.row( 6 * i + 3 * j + 0 ) <<
        l1(1) * l2(2) * l3(0) - l1(2) * l2(1) * l3(0),
        l1(1) * l2(2) * l3(1) - l1(2) * l2(1) * l3(1),
        l1(1) * l2(2) * l3(2) - l1(2) * l2(1) * l3(2),
        l1(1) * c3 - l1(2) * c2;
      M.row( 6 * i + 3 * j + 1 ) <<
        l1(2) * l2(0) * l3(0) - l1(0) * l2(2) * l3(0),
        l1(2) * l2(0) * l3(1) - l1(0) * l2(2) * l3(1),
        l1(2) * l2(0) * l3(2) - l1(0) * l2(2) * l3(2),
        l1(2) * c1 - l1(0) * c3;
      M.row( 6 * i + 3 * j + 2 ) <<
        l1(0) * l2(1) * l3(0) - l1(1) * l2(0) * l3(0),
        l1(0) * l2(1) * l3(1) - l1(1) * l2(0) * l3(1),
        l1(0) * l2(1) * l3(2) - l1(1) * l2(0) * l3(2),
        l1(0) * c2 - l1(1) * c1;
    }
  }

  A = M.topLeftCorner( 6 * nfeat, 3 );
  b = - M.col(3);
  At = A.transpose();
  AtAi = ( At * A ).inverse();
  Vector3d t = AtAi * At * b;

  Vector6d wt;
  wt.head(3) = w;
  wt.tail(3) = t;

  T[0] = gc_wt_to_Rt( wt );

  return 1;
}

//////////////////////////////////////////////////////////////////////////////

void SLAM::motion_only_ba( ob_map obs0, ob_map obs1, vector<int> comm_feat,
    map<int, Vector6d> lines, pose_t & T, vector<int> & inliers ) {

  vector<int>       vec_camera_index;
  vector<int>       vec_fixed_index;
  vector<int>       vec_line_index;
  vector<Vector8d>  vec_observations;
  vector<Vector6d>  vec_camera_param;
  vector<Vector4d>  vec_line_param;

  int fidx = 0;

  vec_camera_param.push_back( gc_Rt_to_wt( T ) );           // 0
  vec_camera_param.push_back( gc_Rt_to_wt( pose_t() ) );    // 1

  for ( size_t i = 0; i < inliers.size(); i++ ) {
    vec_camera_index.push_back(0);
    vec_line_index.push_back(fidx);
    vec_fixed_index.push_back(0);
    vec_fixed_index.push_back(1);
    vec_observations.push_back( obs1[ inliers[i] ] );

    vec_camera_index.push_back(1);
    vec_line_index.push_back(fidx);
    vec_fixed_index.push_back(1);
    vec_fixed_index.push_back(1);
    vec_observations.push_back( obs0[ inliers[i] ] );
    vec_line_param.push_back( gc_av_to_orth(lines[ inliers[i] ]) );
    fidx++;
  }

  //------------------------------------------------------------------------//

  int num_cameras = vec_camera_param.size();
  int num_lines = vec_line_param.size();

  int num_parameters = 6 * num_cameras + 4 * num_lines;

  int num_observations = vec_observations.size();

  int* line_index = new int[num_observations];
  int* camera_index = new int[num_observations];
  int* fixed_index  = new int[2 * num_observations];
  double* observations = new double[8 * num_observations];
  double* parameters = new double[num_parameters];

  for ( int i = 0; i < num_observations; ++i ) {
    line_index[i] = vec_line_index[i];
    camera_index[i] = vec_camera_index[i];
    fixed_index[2*i] = vec_fixed_index[2*i];
    fixed_index[2*i + 1] = vec_fixed_index[2*i + 1];
    for ( int j = 0; j < 8; ++j )
      observations[ 8 * i + j ] = vec_observations[i](j);
  }

  for ( int i = 0; i < num_cameras; ++i )
    for ( int j = 0; j < 6; ++j )
      parameters[ 6 * i + j ] = vec_camera_param[i](j);

  for ( int i = 0; i < num_lines; ++i )
  for ( int j = 0; j < 4; ++j )
    parameters[ 6 * num_cameras + 4 * i + j ] = vec_line_param[i](j);

  //------------------------------------------------------------------------//

  ceres::lba_param_t param;
  param.num_cameras = num_cameras;
  param.num_lines = num_lines;
  param.num_observations = num_observations;
  param.num_iterations = FLAGS_max_num_iter;
  param.num_parameters = num_parameters;
  param.mode = MODE_SPARSE_SCHUR;

  ceres::LBAProblem ba_problem( param );
  ba_problem.set_line_index( line_index );
  ba_problem.set_camera_index( camera_index );
  ba_problem.set_fixed_index( fixed_index );
  ba_problem.set_observations( observations );
  ba_problem.set_parameters( parameters );

  ceres::Problem problem;
  ba_problem.build( &problem );
  ceres::Solver::Options options;
  ba_problem.set_options( &options );
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

//   cout << "\nnum_iter_moba = " << summary.num_successful_steps +
//     summary.num_unsuccessful_steps << " ";

  //------------------------------------------------------------------------//

  Vector6d wt;
  for ( int i = 0; i < 6; ++i )
    wt(i) = parameters[i];

  T = gc_wt_to_Rt( wt );
}

//////////////////////////////////////////////////////////////////////////////
// Transform from init_pose to curr_pose

Vector6d SLAM::line_transform( landmark_t * lm, pose_t T2 ) {
  Vector6d l1 = lm->line;
  pose_t T1 = kfs[ lm->init_kfid ]->T;
  Vector6d l0 = gc_line_from_pose( l1, T1 );
  Vector6d l2 = gc_line_to_pose( l0, T2 );

  return l2;
}

//////////////////////////////////////////////////////////////////////////////

float SLAM::reprojection_error( Vector8d ft, pose_t T, Vector6d line ) {

  float error = 0;

  Vector3d cp, dv;
  cp = line.head(3);
  dv = line.tail(3);

  Vector3d cpc, dvc;
  Vector3d nc;
  Vector3d p1, p2;

  for ( int i = 0; i < 2; ++i ) {
    if ( i == 0 ) {
      p1 << ft[0], ft[1], 1;
      p2 << ft[2], ft[3], 1;
    } else {
      T.t(0) -= baseline;
      p1 << ft[4], ft[5], 1;
      p2 << ft[6], ft[7], 1;
    }

    cpc = gc_point_to_pose( T, cp );
    dvc = T.R * dv;

    nc = cpc.cross(dvc);

    float sql = nc.head(2).norm();
    nc /= sql;

    error += fabs( nc.dot(p1) );
    error += fabs( nc.dot(p2) );
  }

  return error / 4.0;
}

//////////////////////////////////////////////////////////////////////////////

void SLAM::add_new_keyframe( bool add_edge ) {

  keyframe_t* kf = new keyframe_t;
  kf->T = curr_pose;

  add_kf_member_lms( kf );
  add_lms();

  int kfid = kfs.size() > 0 ? kfs.rbegin()->first : -1;

  if ( add_edge ) {
    edge_t e( curr_pose );
    edges.insert( make_pair( pii( kfid, kfid + 1 ), e ) );
    edges.insert( make_pair( pii( kfid + 1, kfid ), e.inverse() ) );
    edge_set.insert( pii( kfid, kfid + 1 ) );

    kf->neighbor_kfs.insert( kfid );
    kfs.rbegin()->second->neighbor_kfs.insert( kfid + 1 );
  }

  kfs.insert( make_pair( kfid + 1, kf ) );

#ifdef DRAW_ALL_VIEW
  vector<Vector8d> temp;
  for ( set<int>::iterator it = final_inliers.begin();
      it != final_inliers.end(); ++it ) {
    temp.push_back( curr_obs[*it] );
  }
  plot.setFrameId( frame_id );
  plot.drawImageTracking( temp );
#endif
}

//////////////////////////////////////////////////////////////////////////////

void SLAM::delete_lms() {

  for ( set<int>::iterator kit = prev_ba_kfs.begin(); kit != prev_ba_kfs.end();
      ++kit ) {
    if ( curr_ba_kfs.find( *kit ) != curr_ba_kfs.end() )
      continue;

    keyframe_t * kf = kfs[ *kit ];
    for ( set<int>::iterator mit = kf->member_lms.begin();
        mit != kf->member_lms.end(); ++mit ) {
      lm_map::iterator lmit = lms.find(*mit);
      if ( lmit == lms.end() ) {
        kf->member_lms.erase( mit );
        continue;
      }

      if ( lmit->second->twice_observed == false ) {
        delete lmit->second;
        lms.erase( lmit );
      }
    }
    //
  }

  prev_ba_kfs = curr_ba_kfs;

}

//////////////////////////////////////////////////////////////////////////////

void SLAM::bundle_adjustment() {
  ////////////////////////////////////////////////////////////////////////////
  // Set

  vector<int>       vec_camera_index;
  vector<int>       vec_fixed_index;
  vector<int>       vec_line_index;
  vector<Vector8d>  vec_observations;
  vector<Vector6d>  vec_camera_param;
  vector<Vector4d>  vec_line_param;

  int kfidx = 0;
  mii kfid_map;
  mii lm_set;
  vector<keyframe_t *> vec_kfs;

  for ( mii::iterator it = ba_kfs.begin(); it != ba_kfs.end(); ++it ) {
    // if ( it->second >= (int) ba_window_size )
    if ( it->second >= (int) FLAGS_ba_window_size )
      continue;

    keyframe_t * kf = kfs[ it->first ];

    for ( set<int>::iterator lmit = kf->member_lms.begin();
        lmit != kf->member_lms.end(); ++lmit ) {

      mii::iterator lit = lm_set.find(*lmit);

      if ( lit == lm_set.end() )
        lm_set.insert( make_pair( *lmit, 1 ) );
      else
        lit->second++;
    }

    vec_kfs.push_back( kf );
    vec_camera_param.push_back( gc_Rt_to_wt( kf->T ) );
    kfid_map.insert( make_pair( it->first, kfidx++ ) );
  }

  int lmidx = 0;
  mii lmid_map;
  vector<landmark_t *> vec_lms;

  for ( mii::iterator lit = lm_set.begin(); lit != lm_set.end(); ++lit ) {
    if ( lit->second < 2 )
      continue;

    lm_map::iterator lmit = lms.find( lit->first );
    if ( lmit == lms.end() )
      continue;
    lmit->second->twice_observed = true;
    lmit->second->ba_updated = true;

    for ( size_t j = 0; j < lmit->second->obs_vec.size(); ++j ) {
      obs_t & obt = lmit->second->obs_vec[j];

      if ( ba_kfs.find( obt.id ) == ba_kfs.end() )
        continue;

      mii::iterator iit = kfid_map.find( obt.id );
      if ( iit == kfid_map.end() ) {
        vec_fixed_index.push_back(1);
        vec_camera_index.push_back( kfidx );

        keyframe_t * kf = kfs[ obt.id ];

        vec_kfs.push_back( kf );
        vec_camera_param.push_back( gc_Rt_to_wt( kf->T ) );
        kfid_map.insert( make_pair( obt.id, kfidx++ ) );
      } else {
        // if ( iit->second < (int) ba_window_size )
        if ( iit->second < (int) FLAGS_ba_window_size )
          vec_fixed_index.push_back(0);
        else
          vec_fixed_index.push_back(1);
        vec_camera_index.push_back( iit->second );
      }

      iit = lmid_map.find( lmit->first );
      if ( iit == lmid_map.end() ) {
        vec_line_index.push_back( lmidx );
        lmid_map.insert( make_pair( lmit->first, lmidx++ ) );
      } else {
        vec_line_index.push_back( iit->second );
      }

      vec_observations.push_back( obt.obs );
    }

    Vector6d line = gc_line_from_pose(
        lmit->second->line, kfs[ lmit->second->init_kfid ]->T );
    vec_line_param.push_back( gc_av_to_orth(line) );
    vec_lms.push_back( lmit->second );
  }

  //  cout << "\t\tkfs/lms:  " << vec_camera_param.size() << "/" << vec_line_param.size() << endl;

  //------------------------------------------------------------------------//

  int num_cameras = vec_camera_param.size();
  int num_lines = vec_line_param.size();
  int num_parameters = 6 * num_cameras + 4 * num_lines;
  int num_observations = vec_observations.size();

  int* line_index = new int[num_observations];
  int* camera_index = new int[num_observations];
  int* fixed_index  = new int[2 * num_observations];
  double* observations = new double[8 * num_observations];
  double* parameters = new double[num_parameters];

  for ( int i = 0; i < num_observations; ++i ) {
    line_index[i] = vec_line_index[i];
    camera_index[i] = vec_camera_index[i];
    fixed_index[2*i] = vec_fixed_index[i];
    fixed_index[2*i + 1] = 0;
    for ( int j = 0; j < 8; ++j )
      observations[ 8 * i + j ] = vec_observations[i](j);
  }

  for ( int i = 0; i < num_cameras; ++i )
    for ( int j = 0; j < 6; ++j )
      parameters[ 6 * i + j ] = vec_camera_param[i](j);

  for ( int i = 0; i < num_lines; ++i )
  for ( int j = 0; j < 4; ++j )
    parameters[ 6 * num_cameras + 4 * i + j ] = vec_line_param[i](j);

  //------------------------------------------------------------------------//

  ceres::lba_param_t param;
  param.num_cameras = num_cameras;
  param.num_lines = num_lines;
  param.num_observations = num_observations;
  param.num_iterations = FLAGS_max_num_iter;
  param.num_parameters = num_parameters;
  param.mode = MODE_SPARSE_SCHUR;

  ceres::LBAProblem ba_problem( param );
  ba_problem.set_line_index( line_index );
  ba_problem.set_camera_index( camera_index );
  ba_problem.set_fixed_index( fixed_index );
  ba_problem.set_observations( observations );
  ba_problem.set_parameters( parameters );

  ceres::Problem problem;
  ba_problem.build( &problem );
  ceres::Solver::Options options;
  ba_problem.set_options( &options );
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  // cout << summary.FullReport() << endl;
  
  // cout << "num_iteration = " << summary.num_successful_steps +
  // summary.num_unsuccessful_steps << endl;
  m_sum_num_iteration += (int)summary.num_successful_steps +
    (int)summary.num_unsuccessful_steps;
  m_sum_final_cost += (double)summary.final_cost;
  m_sum_init_cost += (double)summary.initial_cost;

  //------------------------------------------------------------------------//
  // Get

  for ( int i = 0; i < num_cameras; ++i ) {
    Vector6d pose;
    for ( int j = 0; j < 6; ++j )
      pose[j] = parameters[ 6 * i + j ];
    vec_kfs[i]->T = gc_wt_to_Rt( pose );
  }

  for ( int i = 0; i < num_lines; ++i ) {
    Vector4d line;
    for ( int j = 0; j < 4; ++j )
      line(j) = parameters[ 6 * num_cameras + 4 * i + j ];
    Vector6d line_w = gc_orth_to_av(line);

    landmark_t * lm = vec_lms[i];
    lm->line = gc_line_to_pose( line_w, kfs[ lm->init_kfid ]->T );
  }

  //
}

//////////////////////////////////////////////////////////////////////////////

void SLAM::extend_end_points() {
  const double threshold = extension_length;
  //  pose_t curr_pose = kfs.rbegin()->second->T;

  for ( lm_map::iterator lmit = lms.begin(); lmit != lms.end(); ++lmit ) {
    landmark_t * lm = lmit->second;

    if ( lm->ba_updated == false || lm->currently_visible != true ) {
      continue;
    }

    Vector3d dv = lm->line.tail(3);
    Vector3d cvn = gc_vec3_normalize( dv );

    if ( gc_angle_normvec( cvn, lm->pvn ) > line_vn_angle_thr ) {
      lm->pvn = cvn;
      lm->tt = Vector2d( 0, 0 );
    }

    // main routine

    pose_t init_pose = kfs[ lm->init_kfid ]->T;
    pose_t relative_pose = gc_T_inv( init_pose );
    Vector6d line2 = gc_line_from_pose( lm->line, init_pose );

    Vector3d pc, nc, vc;
    pc = line2.head(3);
    vc = line2.tail(3);
    nc = pc.cross(vc);
    Matrix4d Lc;
    Lc << gc_skew_symmetric(nc), vc, -vc.transpose(), 0;

    obs_t obt = lm->obs_vec.back();
    Vector3d p11 = Vector3d(obt.obs(0), obt.obs(1), 1.0);
    Vector3d p21 = Vector3d(obt.obs(2), obt.obs(3), 1.0);
    Vector2d ln = ( p11.cross(p21) ).head(2);
    ln = ln / ln.norm();

    Vector3d p12 = Vector3d(p11(0) + ln(0), p11(1) + ln(1), 1.0);
    Vector3d p22 = Vector3d(p21(0) + ln(0), p21(1) + ln(1), 1.0);
    Vector3d cam = Vector3d( 0, 0, 0 );

    Vector4d pi1 = gc_ppp_pi(cam, p11, p12);
    Vector4d pi2 = gc_ppp_pi(cam, p21, p22);
    Vector4d e1 = Lc * pi1;
    Vector4d e2 = Lc * pi2;

    Vector3d p0 = gc_plucker_origin( nc, vc );
    Vector3d vn = vc / vc.norm();

    double p0_dist = p0.norm();
    if ( p0_dist > threshold ) {
      lm->ba_updated = false;
      continue;
    }

    Vector3d pc1 = e1.head(3) / e1(3);
    Vector3d pc2 = e2.head(3) / e2(3);
    if ( pc1(2) < 0 || pc2(2) < 0 ) {
      lm->ba_updated = false;
      continue;
    }

    double t1 = vn.dot( pc1 - p0 );
    double t2 = vn.dot( pc2 - p0 );

    Vector2d tt;
    if (t2 > t1)
      tt << t1, t2;
    else
      tt << t2, t1;

    double extend_dist = sqrt( threshold * threshold - p0_dist * p0_dist );

    if ( fabs( tt(0) ) > extend_dist )
      tt(0) = ( tt(0) / fabs( tt(0) ) ) * extend_dist;
    if ( fabs( tt(1) ) > extend_dist )
      tt(1) = ( tt(1) / fabs( tt(1) ) ) * extend_dist;
    if ( tt(0) == tt(1) ) {
      lm->ba_updated = false;
      continue;
    }

    Vector3d init_v = lm->line.tail(3);
    Vector2d tt1 = gc_tt_to_pose( lm->tt, init_v, relative_pose );

    if ( tt1(0) == 0 && tt1(1) == 0 )
      tt1 = tt;
    else
    {
      if (tt(0) < tt1(0))
        tt1(0) = tt(0);

      if (tt(1) > tt1(1))
        tt1(1) = tt(1);
    }

    lm->tt = gc_tt_from_pose( tt1, init_v, relative_pose );

    // printf("%d: tt(0) = %g, tt(1) = %g\n", lmit->first, tt(0), tt(1));

    /////////////////////////////////////////////////////////////////////////

    lm->ba_updated = false;
  }
}

//////////////////////////////////////////////////////////////////////////////

bool SLAM::place_recognized() {
//  if ( pr.pr_DB_query( frame_id, kfs.rbegin()->first, pr_observations,
//        lc_kf_id, match_result ) ) {
//    mii temp = match_result;
//    match_result.clear();
//    for ( mii::iterator it = temp.begin(); it != temp.end(); ++it ) {
//      if ( final_inliers.find( it->first ) == final_inliers.end() )
//        continue;
//      match_result.insert( *it );
//    }
//    return true;
//  }
//  else {
//    return false;
//  }
  return false;
}

//////////////////////////////////////////////////////////////////////////////

bool SLAM::loop_closure() {
  lc_cnt++;
  cout << "\tlc_cnt: " << lc_cnt << "/" << kfs.size() << endl;
  //  cout << "\tlc_kf_id: " << lc_kf_id << endl;

  //  // Plot multiple loop closure
  //  FILE * fp = fopen( "lc.txt", "a" );
  //  fprintf( fp, "%d\t%d\n", kfs.rbegin()->first, lc_kf_id );
  //  fclose( fp );

  me_map mes;
  metric_embedding( lc_kf_id, mes );

  ob_map obs0, obs1;
  for ( mii::iterator it = match_result.begin(); it != match_result.end();
      ++it ) {
    int cid = it->first;  // current
    int lid = it->second; // lc

    if ( lms.find( lid ) == lms.end() )
      continue;

    landmark_t * lm = lms[lid];

    for ( size_t j = 0; j < lm->obs_vec.size(); ++j ) {
      obs_t & obt = lm->obs_vec[j];
      if ( obt.id == lc_kf_id ) {
        obs0.insert( make_pair( lid, obt.obs ) );
        obs1.insert( make_pair( lid, curr_obs[cid] ) ); // lc_kf_id

        break;
      }
    }
    //
  }

  //------------------------------------------------------------------------//

  pose_t best_motion = pose_t();

  if ( pose_estimation( obs0, obs1, best_motion ) == false )
    return false;

  int kfid = kfs.rbegin()->first;
  edge_t e( best_motion );
  edges.insert( make_pair( pii( lc_kf_id, kfid ), e ) );
  edges.insert( make_pair( pii( kfid, lc_kf_id ), e.inverse() ) );
  edge_set.insert( pii( lc_kf_id, kfid ) );

  kfs[ lc_kf_id ]->neighbor_kfs.insert(    kfid );
  kfs[    kfid ]->neighbor_kfs.insert( lc_kf_id );

  //------------------------------------------------------------------------//

  mii temp;

  for ( mii::iterator it = match_result.begin(); it != match_result.end();
      ++it ) {
    int cid = it->first;  // current
    int lid = it->second; // lc

    if ( obs1.find( lid ) == obs1.end() )
      continue;

    if ( lms.find( cid ) == lms.end() )
      continue;

    if ( final_inliers.find( lid ) == final_inliers.end() )
      continue;

    lm_map::iterator lit = lms.find(lid);
    lm_map::iterator cit = lms.find(cid);

    for ( size_t i = 0; i < cit->second->obs_vec.size(); ++i ) {
      obs_t obt = cit->second->obs_vec[i];
      lit->second->obs_vec.push_back( obt );

      set<int>::iterator sit = kfs[obt.id]->member_lms.find( cit->first );
      if ( sit != kfs[obt.id]->member_lms.end() ) {
        kfs[obt.id]->member_lms.erase( sit );
        kfs[obt.id]->member_lms.insert( lid );
      }
    }

    delete cit->second;
    lms.erase( cit );

    temp.insert( make_pair( lid, cid ) );
  }

  for ( set<int>::iterator it = final_inliers.begin();
      it != final_inliers.end(); ++it ) {

    int cid = temp[ *it ];
    Vector8d v = curr_obs[ cid ];
    curr_obs.erase( cid );
    curr_obs.insert( make_pair( *it, v ) );

    //    match_lookup.insert( make_pair( *it, cid ) );
    match_lookup.insert( make_pair( cid, *it ) );
  }

  return true;
}

//////////////////////////////////////////////////////////////////////////////

bool SLAM::consistency_broken() {
  //  return true;

  for ( set<pii>::iterator it = edge_set.begin(); it != edge_set.end(); ++it ){
    pii pair = (*it);
    int n1 = pair.first;
    int n2 = pair.second;

    pose_t T = gc_T_21( kfs[n2]->T, kfs[n1]->T );
    pose_t C = edges[ pair ].C;
    pose_t d = gc_T_21( T, C );

    if ( gc_Rodriguez( d.R ).norm() > kf_rot_thr || d.t.norm() > kf_tr_thr )
      return true;
  }

  return false;
}

//////////////////////////////////////////////////////////////////////////////

void SLAM::pose_optimization() {
  stop_watch.proc_3_tick();

  me_map mes;
  metric_embedding( kfs.rbegin()->first, mes );

  vector<int> pose_vec_1;
  vector<int> pose_vec_2;
  vector<pose_t> ctrs_vec;

  //  cout << "\tkfs/edges: " << kfs.size() << "/" << edge_set.size() << endl;

  int edge_size = 0;
  for ( set<pii>::iterator it = edge_set.begin(); it != edge_set.end(); ++it ){
    pii pair = (*it);
    int n1 = pair.first;
    int n2 = pair.second;

    pose_vec_1.push_back( n1 );
    pose_vec_2.push_back( n2 );
    ctrs_vec.push_back( edges[ pair ].C );

    edge_size++;
  }

  int kfs_size = kfs.size();
  int * pose_index_1 = new int[ edge_size ];
  int * pose_index_2 = new int[ edge_size ];
  double * constraints = new double[ edge_size * 6 ];
  double * parameters  = new double[ kfs_size  * 6 ];

  for ( int i = 0; i < edge_size; ++i ) {
    pose_index_1[i] = pose_vec_1[i];
    pose_index_2[i] = pose_vec_2[i];

    Vector6d temp = gc_Rt_to_wt( ctrs_vec[i] );
    for ( int j = 0; j < 6; ++j )
      constraints[ 6 * i + j ] = temp(j);
  }

  for ( int i = 0; i < kfs_size; ++i ) {
    Vector6d temp = gc_Rt_to_wt( kfs[i]->T );
    for ( int j = 0; j < 6; ++j )
      parameters[ 6 * i + j ] = temp[j];
  }

  cout << "\tStarting PO...\t\t";
  ceres::POProblem po_problem( edge_size, 10 );
  po_problem.set_pose_index_1( pose_index_1 );
  po_problem.set_pose_index_2( pose_index_2 );
  po_problem.set_constraints( constraints );
  po_problem.set_parameters( parameters );
  ceres::Problem problem;
  po_problem.build( &problem );
  ceres::Solver::Options options;
  po_problem.set_options( &options );
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  cout << "\tFinishing PO...\n";

  for ( int i = 0; i < kfs_size; ++i ) {
    Vector6d temp;
    for ( int j = 0; j < 6; ++j )
      temp(j) = parameters[ 6 * i + j ];
    kfs[i]->T = gc_wt_to_Rt( temp );
  }

  for ( set<pii>::iterator it = edge_set.begin(); it != edge_set.end(); ++it ){
    pii pair = (*it);
    int n1 = pair.first;
    int n2 = pair.second;

    edges[ pair ].T = gc_T_21( kfs[n2]->T, kfs[n1]->T );
    edges[ pii( n2, n1 ) ].T = gc_T_21( kfs[n1]->T, kfs[n2]->T );
  }
  //
  stop_watch.proc_3_tock();
}

//////////////////////////////////////////////////////////////////////////////

void SLAM::metric_embedding( int id, me_map & mes ) {
  //  stop_watch.proc_2_tick();

  me_map m;
  m.insert( make_pair( 0.0, id ) );
  kfs[id]->T = pose_t();

  set<int> embedded_kfs;
  embedded_kfs.insert( id );
  int prev_kfid = -1;

  int total_cnt = 0;

  while ( m.size() != 0 ) {
    me_map::iterator mit = m.begin();
    double d = mit->first;
    int start_kfid = mit->second;
    keyframe_t * kf = kfs[start_kfid];
    pose_t T = kf->T;

    mes.insert( make_pair( mit->first, mit->second ) );
    m.erase( mit );

    for ( set<int>::iterator sit = kf->neighbor_kfs.begin();
        sit != kf->neighbor_kfs.end(); ++sit ) {

      total_cnt++;

      int end_kfid = * sit;

      if ( end_kfid == prev_kfid )
        continue;

      if ( embedded_kfs.find( end_kfid ) != embedded_kfs.end() )
        continue;

      pose_t new_T = edges[ pii( start_kfid, end_kfid ) ].T;
      double new_d = new_T.t.norm();

      m.insert( make_pair( d + new_d, end_kfid ) );
      kfs[end_kfid]->T = gc_T_20( new_T, T );

      embedded_kfs.insert( end_kfid );
    }

    prev_kfid = start_kfid;
  }
  //
  //  stop_watch.proc_2_tock();
}

//////////////////////////////////////////////////////////////////////////////

void SLAM::local_bundle_adjustment() {
  me_map mes;
  metric_embedding( kfs.rbegin()->first, mes );

  size_t ba_size = 0;

  for ( me_map::iterator mit = mes.begin(); mit != mes.end(); ++mit ) {
    ba_kfs.insert( make_pair( mit->second, ba_size ) );

    // if ( ++ba_size == 2 * ba_window_size )
    if ( ++ba_size == 2 * FLAGS_ba_window_size )
      break;
  }

  stop_watch.proc_2_tick();
  bundle_adjustment();
  stop_watch.proc_2_tock();

  //------------------------------------------------------------------------//

  vector<int> temp;
  for ( mii::iterator mit = ba_kfs.begin(); mit != ba_kfs.end(); ++mit ) {
    // if ( mit->second >= (int) ba_window_size )
    if ( mit->second >= (int) FLAGS_ba_window_size )
      continue;
    temp.push_back( mit->first );
  }

  vector<pii> vp;
  for ( size_t i = 0; i < temp.size(); ++i )
    for ( size_t j = i + 1; j < temp.size(); ++j )
      vp.push_back( pii( temp[i], temp[j] ) );  // <-- all possible pair

  for ( size_t i = 0; i < vp.size(); ++i ) {
    if ( edges.find( vp[i] ) == edges.end() )
      continue;                                 // <-- pass invalid edge

    int n1 = vp[i].first;
    int n2 = vp[i].second;

    pose_t T = gc_T_21( kfs[n2]->T, kfs[n1]->T );
    edges[ pii( n1, n2 ) ].T = T;
    edges[ pii( n1, n2 ) ].C = T;
    T = gc_T_inv( T );
    edges[ pii( n2, n1 ) ].T = T;
    edges[ pii( n2, n1 ) ].C = T;
  }

  delete_lms();
  extend_end_points();

  //------------------------------------------------------------------------//

  pose_t relative_T = gc_T_21( kfs.rbegin()->second->T, kfs[0]->T );
  relative_T = gc_T_inv( relative_T );

  cout << "\t" << gc_Rt_to_wt( relative_T ).transpose() << endl;
}

//////////////////////////////////////////////////////////////////////////////

void SLAM::save_landmark() {
  vector<Vector6d> lines;
  for ( lm_map::iterator it = lms.begin(); it != lms.end(); ++it ) {
    landmark_t * lm = it->second;
//    if ( lm->twice_observed == false )
//      continue;
//    if ( fabs( lm->tt(0) - lm->tt(1) ) < 1.0 )
//      continue;

    Vector6d line = lm->line;
    Vector3d p, n, v;
    p = line.head(3);
    v = line.tail(3);
    n = p.cross(v);
    Vector3d p0 = v.cross(n) / v.dot(v);
    Vector3d vn = v / v.norm();

    pose_t T = kfs[ lm->init_kfid ]->T;

    Vector3d p1c = p0 + vn * lm->tt(0);
    Vector3d p1w = gc_poit_from_pose( T, p1c );
    Vector3d p2c = p0 + vn * lm->tt(1);
    Vector3d p2w = gc_poit_from_pose( T, p2c );

    line << p1w(0), p1w(1), p1w(2), p2w(0), p2w(1), p2w(2);
    lines.push_back( line );
  }

  char filename[512];
  sprintf(filename,
      "/home/spacetrain/workspace3/sl_slam/data/simulation/house/"
      "landmark_basize%d_maxnumiter%d.txt",
      (int)FLAGS_ba_window_size, (int)FLAGS_max_num_iter);
  ofstream fout(filename, ios::out);
  for(int i=0; i<lines.size(); ++i) {
    Vector6d line = lines[i];
    fout << line(2) << "\t" << -line(1) << "\t" << line(0) << "\t"
      << line(5) << "\t" << -line(4) << "\t" << line(3) << endl;
  }
  fout.close();
}

void SLAM::save_trajectory() {

  me_map mes;
  metric_embedding( 0, mes );

  vector<pose_t> trajectory;
  for ( kf_map::iterator it = kfs.begin(); it != kfs.end(); ++it )
    trajectory.push_back( gc_T_inv( it->second->T ) );

  char filename[512];
  sprintf(filename,
      "/home/spacetrain/workspace3/sl_slam/data/trajectory/traj_slslam_basize%d.txt",
      // (int)ba_window_size);
      (int)FLAGS_ba_window_size);

  ofstream fout(filename, ios::out);
  for(int i=0; i<trajectory.size(); ++i) {
    pose_t traj = trajectory[i];
    Vector3d rot = gc_Rodriguez(traj.R);
    fout << i << "\t" << traj.t(2) << "\t" << -traj.t(0) << "\t" <<
      -traj.t(1) << "\t" << rot(0) << "\t" << rot(1) << "\t" << rot(2) << endl;
  }
  fout.close();
}

void SLAM::draw() {
  vector<pose_t> trajectory;
  vector<Vector6d> lines;

  me_map mes;
  metric_embedding( 0, mes );

  for ( kf_map::iterator it = kfs.begin(); it != kfs.end(); ++it )
    trajectory.push_back( gc_T_inv( it->second->T ) );

  for ( lm_map::iterator it = lms.begin(); it != lms.end(); ++it ) {
    landmark_t * lm = it->second;
    if ( lm->twice_observed == false )
      continue;
    if ( fabs( lm->tt(0) - lm->tt(1) ) < 1.0 )
      continue;

    Vector6d line = lm->line;
    Vector3d p, n, v;
    p = line.head(3);
    v = line.tail(3);
    n = p.cross(v);
    Vector3d p0 = v.cross(n) / v.dot(v);
    Vector3d vn = v / v.norm();

    pose_t T = kfs[ lm->init_kfid ]->T;

    Vector3d p1c = p0 + vn * lm->tt(0);
    Vector3d p1w = gc_poit_from_pose( T, p1c );
    Vector3d p2c = p0 + vn * lm->tt(1);
    Vector3d p2w = gc_poit_from_pose( T, p2c );

    line << p1w(0), p1w(1), p1w(2), p2w(0), p2w(1), p2w(2);
    lines.push_back( line );
  }

  plot.setFrameId( frame_id );
  plot.drawResult( trajectory, lines );

  map<int, Vector8d>::const_iterator it;
  vector<Vector8d> obs0, obs1;
  vector<int> segids0, segids1;
  for(it=prev_kf_obs.begin(); it!=prev_kf_obs.end(); ++it) {
    obs0.push_back(it->second);
    segids0.push_back(it->first);
  }
  for(it=curr_obs.begin(); it!=curr_obs.end(); ++it) {
    obs1.push_back(it->second);
    segids1.push_back(it->first);
  }
  plot.drawObservation(segids0, segids1, obs0, obs1);
}

//////////////////////////////////////////////////////////////////////////////

void SLAM::end_slam_cycle() {
  prev_kf_obs = curr_obs;
}

//////////////////////////////////////////////////////////////////////////////

void SLAM::time_tag(  string str  ) {
  cout << "\tTime(" << str << "): " << stop_watch.read_current_time() << endl;
}

//////////////////////////////////////////////////////////////////////////////

void SLAM::post_processing() {
  time_statistics ts;

  stop_watch.proc_1_statistics( ts );
  cout << "\tProc_1: " << ts.t / ts.c << endl;

  stop_watch.proc_2_statistics( ts );
  cout << "\tProc_2: " << ts.t / ts.c << endl;

  stop_watch.proc_3_statistics( ts );
  cout << "\tProc_3: " << ts.t / ts.c << endl;

  //  ////////////////////////////////////////////////////////////////////////////
  //  // LOG at end
  //
  //  pose_optimization();
  //  me_map mes;
  //  metric_embedding( 0, mes );
  //
  //  FILE * fp = fopen( "tr.txt", "w" );
  //  for ( kf_map::iterator it = kfs.begin(); it != kfs.end(); ++it ) {
  //    pose_t T = gc_T_inv( it->second->T );
  //    fprintf( fp, "%f\t%f\t%f\n", T.t(0), T.t(1), T.t(2) );
  //  }
  //  fclose( fp );
  //
  //  fp = fopen( "line.txt", "w" );
  //  for ( lm_map::iterator it = lms.begin(); it != lms.end(); ++it ) {
  //    landmark_t * lm = it->second;
  //    if ( lm->twice_observed == false )
  //      continue;
  //
  //    if ( fabs( lm->tt(0) - lm->tt(1) ) < 1.0 )
  //      continue;
  //
  //    Vector6d line = lm->line;
  //    Vector3d p, n, v;
  //    p = line.head(3);
  //    v = line.tail(3);
  //    n = p.cross(v);
  //    Vector3d p0 = v.cross(n) / v.dot(v);
  //    Vector3d vn = v / v.norm();
  //
  //    pose_t T = kfs[ lm->init_kfid ]->T;
  //
  //    Vector3d p1c = p0 + vn * lm->tt(0);
  //    Vector3d p1w = gc_poit_from_pose( T, p1c );
  //    Vector3d p2c = p0 + vn * lm->tt(1);
  //    Vector3d p2w = gc_poit_from_pose( T, p2c );
  //
  //    fprintf( fp, "%f\t%f\t%f\n", p1w(0), p1w(1), p1w(2) );
  //    fprintf( fp, "%f\t%f\t%f\n", p2w(0), p2w(1), p2w(2) );
  //  }
  //  fclose( fp );
  //
  //  ////////////////////////////////////////////////////////////////////////////

  m_total_time = stop_watch.read_current_time();
  cout << "\n\n----------------------------------------" << endl;
  cout << "Total time: " << m_total_time << endl;
  cout << "KF    size: " << kfs.size() << endl;
  cout << "LM    size: " << lms.size() << endl;
  cout << "Edge  size: " << edges.size() / 2 << endl;
  cout << "LC     cnt: " << lc_cnt << endl;
  //  cout << "FEAT:      " << feat_total * 1.0 / feat_size << endl;
  //  cout << feat_total << " " << feat_size << endl;

  cout << endl;
}

//////////////////////////////////////////////////////////////////////////////
