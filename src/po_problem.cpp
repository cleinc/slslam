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

#include "po_problem.h"

namespace ceres {

POProblem::POProblem( int a, int n ) {
  num_threads = 1;
  eta = 1e-2;
  robustify = false;

  set_size( a );
  set_num_iterations( n );
}

POProblem::~POProblem() {
  delete []pose_index_1_;
  delete []pose_index_2_;
  delete []constraints_;
  delete []parameters_;
}

void POProblem::build( Problem * problem ) {
  const int _pose_block_size = pose_block_size();
  const double* _constraints = constraints();

  for (int i = 0; i < num_size(); ++i) {
    CostFunction* cost_function =
        new AutoDiffCostFunction<PoseConstraintError, 6, 6, 6>(
            new PoseConstraintError(  _constraints[6 * i + 0],
                                      _constraints[6 * i + 1],
                                      _constraints[6 * i + 2],
                                      _constraints[6 * i + 3],
                                      _constraints[6 * i + 4],
                                      _constraints[6 * i + 5] ) );

    // If enabled use Huber's loss function.
    LossFunction* loss_function = robustify ? new HuberLoss(0.001) : NULL;

    double * pose1 = parameters() + _pose_block_size * pose_index_1()[i];
    double * pose2 = parameters() + _pose_block_size * pose_index_2()[i];

    problem->AddResidualBlock( cost_function, loss_function, pose1, pose2 );

    if ( i == 0 )
      problem->SetParameterBlockConstant( pose1 );
  }
}

void POProblem::set_options( Solver::Options* options ) {
  options->linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options->num_linear_solver_threads = num_threads;

  options->max_num_iterations = num_iterations;
  options->minimizer_progress_to_stdout = true;
  options->num_threads = num_threads;
  options->eta = eta;

  options->logging_type = SILENT;
}

}  // namespace ceres
