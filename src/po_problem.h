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

#ifndef PO_PROBLEM_H_
#define PO_PROBLEM_H_

#include <string>
#include "ceres/ceres.h"
#include "ceres/rotation.h"

template <typename T>
void gc_T_inv( T P[6], T Pi[6] ) {
  Pi[0] = - P[0];
  Pi[1] = - P[1];
  Pi[2] = - P[2];

  T v[3];
  v[0] = - P[3];
  v[1] = - P[4];
  v[2] = - P[5];

  ceres::AngleAxisRotatePoint( Pi, v, Pi + 3 );
}

// R20 = R21 * R10
template <typename T>
void gc_w_20( T w21[3], T w10[3], T w20[3] ) {

  T q21[4], q10[4], q20[4];

  ceres::AngleAxisToQuaternion( w21, q21 );
  ceres::AngleAxisToQuaternion( w10, q10 );

  ceres::QuaternionProduct( q21, q10, q20 );
  ceres::QuaternionToAngleAxis( q20, w20 );
}

// T20 = T21 * T10
template <typename T>
void gc_T_20( T T21[6], T T10[6], T T20[6] ) {
  gc_w_20( T21, T10, T20 );

  ceres::AngleAxisRotatePoint( T21, T10 + 3, T20 + 3 );

  T20[3] += T21[3];
  T20[4] += T21[4];
  T20[5] += T21[5];
}

namespace ceres {

struct PoseConstraintError {
  PoseConstraintError(double wo0, double wo1, double wo2,
                       double to0, double to1, double to2 )
      : wo0(wo0), wo1(wo1), wo2(wo2), to0(to0), to1(to1), to2(to2) {}

  template <typename T>
  bool operator()(const T* const pose1, const T* const pose2,
                  T* residuals) const {
    T T1[6], T2[6], C[6], Tc[6], Te[6], T2i[6];

    // Tc = C * T1;
    // Te = T2^-1 * Tc;

    for ( int i = 0; i < 6; ++i ) {
      T1[i] = pose1[i];
      T2[i] = pose2[i];
    }

    C[0] = (T) wo0;
    C[1] = (T) wo1;
    C[2] = (T) wo2;
    C[3] = (T) to0;
    C[4] = (T) to1;
    C[5] = (T) to2;

    gc_T_20( C, T1, Tc );
    gc_T_inv( T2, T2i );
    gc_T_20( T2i, Tc, Te );

    residuals[0] = Te[0];
    residuals[1] = Te[1];
    residuals[2] = Te[2];
    residuals[3] = Te[3];
    residuals[4] = Te[4];
    residuals[5] = Te[5];

    return true;
  }

  double  wo0, wo1, wo2, to0, to1, to2;
};

class POProblem {
 public:
  explicit POProblem( int s, int n );
  ~POProblem();

  int pose_block_size()               const { return 6;             }
  int num_size()                      const { return size_;         }
  const int* pose_index_1()           const { return pose_index_1_; }
  const int* pose_index_2()           const { return pose_index_2_; }
  const double* constraints()         const { return constraints_; }
  double* parameters()                const { return parameters_;   }

  inline void set_size( int s )             { size_ = s;            }
  inline void set_num_iterations( int s )   { num_iterations = s;   }
  inline void set_pose_index_1( int* idx )  { pose_index_1_ = idx;  }
  inline void set_pose_index_2( int* idx )  { pose_index_2_ = idx;  }
  inline void set_constraints( double* d )  { constraints_ = d;    }
  inline void set_parameters( double* d )   { parameters_ = d;      }

  void build( Problem* problem );
  void set_options( Solver::Options* options );

 private:
  int num_iterations;
  int num_threads;
  double eta;
  bool robustify;

  int size_;

  int* pose_index_1_;
  int* pose_index_2_;
  double* constraints_;
  double* parameters_;
};

}  // namespace ceres

#endif  // PO_PROBLEM_H_
