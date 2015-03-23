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

#ifndef LBA_PROBLEM_H_
#define LBA_PROBLEM_H_

#include <iostream>
#include <string>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include <cstdio>
#include <cstdlib>

#include "gflags/gflags.h"

#define MODE_SPARSE_SCHUR            1
#define MODE_SPARSE_NORMAL_CHOLESKY  2

namespace ceres {

// Templated pinhole camera model for used with Ceres.  The camera is
// parameterized using 9 parameters: 3 for rotation, 3 for translation, 1 for
// focal length and 2 for radial distortion. The principal point is not modeled
// (i.e. it is assumed be located at the image center).
struct LineReprojectionError {
  LineReprojectionError(double x0, double y0, double x1, double y1,
      double x2, double y2, double x3, double y3 )
      : x0(x0), y0(y0), x1(x1), y1(y1), x2(x2), y2(y2), x3(x3), y3(y3) {}

  template <typename T>
  bool operator()(const T* const camera,
                  const T* const line,
                  T* residuals) const {
    T a, b, g, t;
    a = line[0];
    b = line[1];
    g = line[2];
    t = line[3];

    T s1 = sin(a);
    T c1 = cos(a);
    T s2 = sin(b);
    T c2 = cos(b);
    T s3 = sin(g);
    T c3 = cos(g);

    T d = cos(t) / sin(t);

    T cp[3], dv[3];
    cp[0] = - (c1 * s2 * c3 + s1 * s3) * d;
    cp[1] = - (c1 * s2 * s3 - s1 * c3) * d;
    cp[2] = - (c1 * c2               ) * d;

    dv[0] = s1 * s2 * c3 - c1 * s3;
    dv[1] = s1 * s2 * s3 + c1 * c3;
    dv[2] = s1 * c2;

    T pc[3], dc[3];
    ceres::AngleAxisRotatePoint(camera, cp, pc);
    ceres::AngleAxisRotatePoint(camera, dv, dc);

    ////////////////////////////////////////////////////////////////////////////
    // Right camera

    pc[0] += camera[3];
    pc[1] += camera[4];
    pc[2] += camera[5];

    T n[3];
    n[0] = pc[1] * dc[2] - pc[2] * dc[1];
    n[1] = pc[2] * dc[0] - pc[0] * dc[2];
    n[2] = pc[0] * dc[1] - pc[1] * dc[0];

    T sql = sqrt( n[0] * n[0] + n[1] * n[1] );
    n[0] /= sql;
    n[1] /= sql;
    n[2] /= sql;

    residuals[0] = - ( T(x0) * n[0] + T(y0) * n[1] + n[2] );
    residuals[1] = - ( T(x1) * n[0] + T(y1) * n[1] + n[2] );

    ////////////////////////////////////////////////////////////////////////////
    // Left camera

    T base_line = T(0.12);

    pc[0] -= base_line;

    n[0] = pc[1] * dc[2] - pc[2] * dc[1];
    n[1] = pc[2] * dc[0] - pc[0] * dc[2];
    n[2] = pc[0] * dc[1] - pc[1] * dc[0];

    sql = sqrt( n[0] * n[0] + n[1] * n[1] );
    n[0] /= sql;
    n[1] /= sql;
    n[2] /= sql;

    residuals[2] = - ( T(x2) * n[0] + T(y2) * n[1] + n[2] );
    residuals[3] = - ( T(x3) * n[0] + T(y3) * n[1] + n[2] );

    return true;
  }

  double x0, y0, x1, y1, x2, y2, x3, y3;
};

typedef struct {
  int num_cameras;
  int num_lines;
  int num_observations;
  int num_iterations;
  int num_parameters;
  int mode;
} lba_param_t;

class LBAProblem {
 public:
  explicit LBAProblem( lba_param_t param );
  ~LBAProblem();

  int camera_block_size()      const { return 6;                 }
  int line_block_size()        const { return 4;                 }
  int num_cameras()            const { return num_cameras_;      }
  int num_lines()              const { return num_lines_;        }
  int num_observations()       const { return num_observations_; }
  int num_parameters()         const { return num_parameters_;   }
  const int* line_index()      const { return line_index_;       }
  const int* camera_index()    const { return camera_index_;     }
  const int* fixed_index()     const { return fixed_index_;      }
  const double* observations() const { return observations_;     }
  const double* parameters()   const { return parameters_;       }
  double* mutable_cameras()          { return parameters_;       }
  double* mutable_lines() {
    return parameters_  + camera_block_size() * num_cameras_;
  }

  inline void set_line_index( int* idx )    { line_index_ = idx;     }
  inline void set_camera_index( int* idx )  { camera_index_ = idx;   }
  inline void set_fixed_index( int* idx )   { fixed_index_ = idx;    }
  inline void set_observations( double* d ) { observations_ = d;     }
  inline void set_parameters( double* d )   { parameters_ = d;       }
  inline void set_logging_type( bool b )    { logging_type = b;      }

  void build( Problem* problem );
  void set_options( Solver::Options* options );

 private:

  inline void set_num_cameras( int s )      { num_cameras_      = s; }
  inline void set_num_lines( int s )        { num_lines_        = s; }
  inline void set_num_observations( int s ) { num_observations_ = s; }
  inline void set_num_iterations( int s )   { num_iterations_   = s; }
  inline void set_num_parameters( int s )   { num_parameters_   = s; }
  inline void set_mode( int s )             { mode_             = s; }

  int mode_;
  int num_cameras_;
  int num_lines_;
  int num_observations_;
  int num_parameters_;
  int num_iterations_;

  std::string input;
  std::string solver_type;
  std::string preconditioner_type;
  int num_threads;
  double eta;
  bool use_schur_ordering;
  bool use_quaternions;
  bool use_local_parameterization;
  bool robustify;
  bool logging_type;

  int* line_index_;
  int* camera_index_;
  int* fixed_index_;
  double* observations_;
  // The parameter vector is laid out as follows
  // [camera_1, ..., camera_n, point_1, ..., point_m]
  double* parameters_;
};

}  // namespace ceres

#endif  // LBA_PROBLEM_H_
