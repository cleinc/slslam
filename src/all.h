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

#ifndef ALL_H_
#define ALL_H_

#include <iostream>
#include <fstream>
#include <algorithm>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "Eigen/Dense"
#include "Eigen/QR"

#include "glog/logging.h"

using namespace std;
using namespace Eigen;

typedef Matrix<double,8,1> Vector8d;
typedef Matrix<double,6,1> Vector6d;

//////////////////////////////////////////////////////////////////////////////
// Data structure

typedef struct _pose_t {
  _pose_t() : R( Matrix3d::Identity() ), t( Vector3d::Zero() ) {}
  _pose_t( const _pose_t & T ) : R( T.R ), t( T.t ) {}
  _pose_t( Matrix3d R, Vector3d t ) : R(R), t(t) {}

  Matrix3d R;
  Vector3d t;
} pose_t;

#endif /* ALL_H_ */
