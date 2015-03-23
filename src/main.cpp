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

DEFINE_int32(ba_window_size, 10, "Local Bundle Adjustment moving window size");
DEFINE_int32(max_num_iter, 10, "Maximum number of iteration in BA");
DEFINE_int32(delay, 5, "Time delay in [ms] for OpenCV visualization");
DEFINE_int32(stopfrm, 99999, "Stop frame number");
DEFINE_int32(rseed, 4, "Random Seed");
DEFINE_bool(robust, true, "Use robust function");

int main(int argc, char *argv[])
{
  google::ParseCommandLineFlags(&argc, &argv, true);

  if ( system( "rm *.txt" ) );
  if ( system( "rm output/tracking/*.*" ) );
  if ( system( "rm output/glscene/*.*" ) );
  if ( system( "rm output/line_tracking/*.*" ) );
  if ( system( "rm output/allview/*.*" ) );

  int frame_id = 0;
  SLAM slam;

  static bool is_first = true;
  char lastkey;
  bool first_loop_closure = false;
  while ( lastkey != 27 ) {
    if ( frame_id > FLAGS_stopfrm)
      break;
    slam.start_slam_cycle( frame_id );
    if ( ! slam.grab_new_frame( frame_id++ ) ) {
      break;
    }
    if ( slam.check_input_data() == 1 )
      continue;

    if ( slam.check_input_data() == 2 ) {
      slam.add_new_keyframe( false );
      slam.end_slam_cycle();
      continue;
    }

    if ( ! slam.check_keyframe_motion() )  // visual odometry
      continue;
    slam.add_new_keyframe( true );
    if ( slam.place_recognized() && slam.loop_closure() ) {
      if ( slam.consistency_broken() ) {
        slam.pose_optimization();
        first_loop_closure = true;
      }
    }
    slam.local_bundle_adjustment();
    slam.draw();
    slam.end_slam_cycle();
    lastkey = cv::waitKey(FLAGS_delay);
    if(is_first) {
      cv::waitKey(0);
      is_first = false;
    }
    if(first_loop_closure)
      break;
  }
  slam.post_processing();
  slam.save_trajectory();
  slam.save_landmark();
  cout << "Average number of iterations = " << (float)slam.m_sum_num_iteration /
    (float)frame_id << endl;
  cout << "Average initial costs = " << slam.m_sum_init_cost /
    (double)frame_id << endl;
  cout << "Average final costs = " << slam.m_sum_final_cost /
    (double)frame_id << endl;
  cv::waitKey(0);
  return 0;
}
