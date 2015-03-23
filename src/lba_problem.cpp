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

#include "lba_problem.h"

DECLARE_bool(robust);

namespace ceres {

LBAProblem::LBAProblem( lba_param_t param ) {
  input = "";
  solver_type = "sparse_schur";
  preconditioner_type = "jacobi";
  num_threads = 1;
  eta = 1e-2;
  use_schur_ordering = false;
  use_quaternions = false;
  use_local_parameterization = false;
  robustify = FLAGS_robust;
  logging_type = false;

  set_num_cameras( param.num_cameras );
  set_num_lines( param.num_lines );
  set_num_observations( param.num_observations );
  set_num_iterations( param.num_iterations );
  set_num_parameters( param.num_parameters );
  set_mode( param.mode );
}

LBAProblem::~LBAProblem() {
  delete []line_index_;
  delete []camera_index_;
  delete []fixed_index_;
  delete []observations_;
  delete []parameters_;
}

void LBAProblem::build( Problem * problem ) {
  const int _line_block_size = line_block_size();
  const int _camera_block_size = camera_block_size();
  double* lines = mutable_lines();
  double* cameras = mutable_cameras();

  const double* _observations = observations();

  for (int i = 0; i < num_observations(); ++i) {
    CostFunction* cost_function;

    cost_function =
        new AutoDiffCostFunction<LineReprojectionError, 4, 6, 4>(
            new LineReprojectionError( _observations[8 * i + 0],
                                       _observations[8 * i + 1],
                                       _observations[8 * i + 2],
                                       _observations[8 * i + 3],
                                       _observations[8 * i + 4],
                                       _observations[8 * i + 5],
                                       _observations[8 * i + 6],
                                       _observations[8 * i + 7] ));

    // If enabled use Huber's loss function.
    // double focal_length = 320.0;
        double focal_length = 406.05;
        LossFunction* loss_function =
        robustify ? new HuberLoss(1.0/focal_length) : NULL;
        //    LossFunction* loss_function = robustify ? new CauchyLoss(1.0) : NULL;

    double* camera = cameras + _camera_block_size * camera_index()[i];
    double* line = lines + _line_block_size * line_index()[i];

    problem->AddResidualBlock(cost_function, loss_function, camera, line);

    if ( fixed_index()[2*i] )
      problem->SetParameterBlockConstant( camera );
    if ( fixed_index()[2*i+1] )
      problem->SetParameterBlockConstant( line );
  }
}

void LBAProblem::set_options( Solver::Options* options ) {
  switch ( mode_ ) {
    case MODE_SPARSE_SCHUR:
      options->linear_solver_type = ceres::SPARSE_SCHUR;
    case MODE_SPARSE_NORMAL_CHOLESKY:
      options->linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  }

  options->num_linear_solver_threads = num_threads;

  options->linear_solver_ordering = new ceres::ParameterBlockOrdering;
  const int _num_lines = num_lines();
  const int _line_block_size = line_block_size();
  double* lines = mutable_lines();
  const int _num_cameras = num_cameras();
  const int _camera_block_size = camera_block_size();
  double* cameras = mutable_cameras();

  // The points come before the cameras.
  for (int i = 0; i < _num_lines; ++i) {
    options->linear_solver_ordering->AddElementToGroup(lines + _line_block_size * i, 0);
  }

  for (int i = 0; i < _num_cameras; ++i) {
    // When using axis-angle, there is a single parameter block for
    // the entire camera.
    options->linear_solver_ordering->AddElementToGroup(cameras + _camera_block_size * i, 0);
  }

  options->max_num_iterations = num_iterations_;
//  cout << " Iterations: " << num_iterations_ << " ";
  options->minimizer_progress_to_stdout = true;
  options->num_threads = num_threads;
  options->eta = eta;

  if ( logging_type == false )
    options->logging_type = SILENT;
}

}  // namespace ceres
