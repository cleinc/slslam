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

#ifndef GC_H_
#define GC_H_

#include "all.h"
#include "Eigen/SVD"

Matrix3d gc_Rodriguez( Vector3d aa );
Vector3d gc_Rodriguez( Matrix3d R );
pose_t gc_T_inv( pose_t T );
Vector3d gc_point_to_pose( pose_t T, Vector3d pt );
Vector3d gc_poit_from_pose( pose_t T, Vector3d pt );
Vector6d gc_line_to_pose(Vector6d line0, pose_t T);
Vector6d gc_line_from_pose(Vector6d line, pose_t T);
Vector4d gc_ppp_pi(Vector3d x1, Vector3d x2, Vector3d x3);
Vector6d gc_pipi_plk( Vector4d pi1, Vector4d pi2);
Vector3d gc_plucker_origin(Vector3d n, Vector3d v);
Matrix3d gc_skew_symmetric( Vector3d v );
Vector2d gc_tt_to_pose( Vector2d tt0, Vector3d v, pose_t T );
Vector2d gc_tt_from_pose( Vector2d tt1, Vector3d v, pose_t T );
Vector6d gc_plk_to_pose( Vector6d plk_w, pose_t T );
Vector6d gc_plk_from_pose( Vector6d plk_c, pose_t T );
Vector3d gc_vec3_normalize( Vector3d v );
double gc_angle_normvec( Vector3d  v1, Vector3d  v2 );
// T_20 = T_21 * T_10
pose_t gc_T_20( pose_t T21, pose_t T10 );
// T_20 = T_21 * T_10 --> T_21 = T_20 * (T_10)^-1
pose_t gc_T_21( pose_t T10, pose_t T20 );
pose_t gc_wt_to_Rt( Vector6d wt );
Vector6d gc_Rt_to_wt( pose_t Rt );

Vector4d gc_av_to_aid(Vector6d av);
Vector6d gc_aid_to_av(Vector4d aid);

Vector4d gc_av_to_asd(Vector6d av);
Vector6d gc_asd_to_av(Vector4d aid);

Vector4d gc_av_to_orth(Vector6d av);
Vector6d gc_orth_to_av(Vector4d orth);

#endif /* GC_H_ */



