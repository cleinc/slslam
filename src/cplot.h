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

#ifndef CPLOT_H_
#define CPLOT_H_

#include <GLFW/glfw3.h>
#include "all.h"

class CPlot {
public:
  CPlot();
  virtual ~CPlot();

  void drawObservation( const vector<int> &segids0, const vector<int> &segids1,
      const vector<Vector8d> &ft0, const vector<Vector8d> &ft1 );
  void saveTrackingView( int i );
  void drawFloorGrid();
  void drawResult( vector<pose_t> trajectory, vector<Vector6d> lines );
  void drawResultTrajectory( vector<pose_t> trajectory );
  void glPrepare( Eigen::Vector3d twc );
  void drawRobot( pose_t T );
  void drawLine( vector<Vector6d> lines );
  void saveGlScene( int idx );
  void drawTrajectory( vector<pose_t> trajectory );
  void drawDropLine( vector<pose_t> trajectory );
  void setDebugPlot( bool v );
  inline void setFrameId( int i ) { frame_id = i; }
  void drawImageTracking( vector<Vector8d> obs );
  void saveAllView();

  bool debug_plot;
  bool save_image;

  int gl_win_width;
  int gl_win_height;
  int frame_id;

  GLFWwindow* window;
};

void plotDrawTracking( Vector8d obs, int line_id, int frame_id );

#endif /* CPLOT_H_ */

