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

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "gc.h"

Matrix3d gc_Rodriguez( Vector3d aa ) {
  double w[3], a[9];
  Map<Matrix3d > R(a);

  for ( int i = 0; i < 3; ++i )
    w[i] = aa[i];

  ceres::AngleAxisToRotationMatrix( w, a );

  return R;
}

Vector3d gc_Rodriguez( Matrix3d R ) {
  double a[9], w[3];

  Matrix<double,9,1> v;
  v << R(0,0), R(1,0), R(2,0), R(0,1), R(1,1), R(2,1), R(0,2), R(1,2), R(2,2);

  for ( int i = 0; i < 9; ++i )
    a[i] = v(i);

  ceres::RotationMatrixToAngleAxis( a, w );

  Map<Vector3d > x(w);
  return x;
}

pose_t gc_T_inv( pose_t T ) {
  return pose_t( T.R.inverse(), - T.R.inverse() * T.t );
}

Vector3d gc_point_to_pose( pose_t T, Vector3d pt_w ) {
  return T.R * pt_w + T.t;
}

Vector3d gc_poit_from_pose( pose_t T, Vector3d pt_c ) {
  return gc_point_to_pose( gc_T_inv( T ), pt_c );
}

Vector6d gc_line_to_pose(Vector6d line_w, pose_t T) {
  Vector6d line_c;

  Vector3d cp0, dv0;
  cp0 = line_w.head(3);
  dv0 = line_w.tail(3);

  Vector3d cp1 = gc_point_to_pose( T, cp0 );
  Vector3d dv1 = T.R * dv0;

  line_c.head(3) = cp1;
  line_c.tail(3) = dv1;

  return line_c;
}

Vector6d gc_line_from_pose(Vector6d line_c, pose_t T) {
  return gc_line_to_pose( line_c, gc_T_inv( T ) );
}

Vector6d gc_plk_to_pose( Vector6d plk_w, pose_t T ) {
  Vector3d nw = plk_w.head(3);
  Vector3d vw = plk_w.tail(3);

  Vector3d nc = T.R * nw + gc_skew_symmetric(T.t) * T.R * vw;
  Vector3d vc = T.R * vw;

  Vector6d plk_c;
  plk_c.head(3) = nc;
  plk_c.tail(3) = vc;
  return plk_c;
}

Vector6d gc_plk_from_pose( Vector6d plk_c, pose_t T ) {
  return gc_plk_to_pose( plk_c, gc_T_inv( T ) );
}

Vector4d gc_ppp_pi(Vector3d x1, Vector3d x2, Vector3d x3) {
  Vector4d pi;
  pi << ( x1 - x3 ).cross( x2 - x3 ), - x3.dot( x1.cross( x2 ) );

  return pi;
}

Vector6d gc_pipi_plk( Vector4d pi1, Vector4d pi2){
  Vector6d plk;
  Matrix4d dp = pi1 * pi2.transpose() - pi2 * pi1.transpose();

  plk << dp(0,3), dp(1,3), dp(2,3), - dp(1,2), dp(0,2), - dp(0,1);
  return plk;
}

Vector3d gc_plucker_origin(Vector3d n, Vector3d v) {
  return v.cross(n) / v.dot(v);
}

Matrix3d gc_skew_symmetric( Vector3d v ) {
  Matrix3d S;
  S << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return S;
}

Vector2d gc_tt_to_pose( Vector2d tt0, Vector3d v, pose_t T ) {
  Vector2d tt1 = Vector2d::Zero();

  if ( tt0(0) == 0 && tt0(1) == 0 )
    return tt1;

  pose_t Ti = gc_T_inv( T );
  Vector3d vn = v / v.norm();
  double offset = Ti.t.dot( vn );

  tt1 = tt0 - Vector2d( offset, offset );

  return tt1;
}

Vector2d gc_tt_from_pose( Vector2d tt1, Vector3d v, pose_t T ) {
  Vector2d tt0;

  pose_t Ti = gc_T_inv( T );
  Vector3d vn = v / v.norm();
  double offset = Ti.t.dot( vn );

  tt0 = tt1 + Vector2d( offset, offset );

  return tt0;
}

Vector3d gc_vec3_normalize( Vector3d v ) {
  double n = v.norm();
  if ( n != 0 )
    v /= n;
  return v;
}

double gc_angle_normvec( Vector3d  v1, Vector3d  v2 ) {
  return acos( v1.dot(v2) );
}

// T20 = T21 * T10
pose_t gc_T_20( pose_t T21, pose_t T10 ) {
  return pose_t( T21.R * T10.R, T21.R * T10.t + T21.t );
}

// T21 = T20 * (T10)^-1
pose_t gc_T_21( pose_t T20, pose_t T10 ) {
  return gc_T_20( T20, gc_T_inv( T10 ) );
}

pose_t gc_wt_to_Rt( Vector6d wt ) {
  Vector3d w = wt.head(3);
  return pose_t( gc_Rodriguez( w ), wt.tail(3) );
}

Vector6d gc_Rt_to_wt( pose_t Rt ) {
  Vector6d wt;
  wt.head(3) = gc_Rodriguez( Rt.R );
  wt.tail(3) = Rt.t;

  return wt;
}

////////////////////////////////////////////////////////////////////////////////

Vector4d gc_av_to_aid(Vector6d av) {

  Vector4d aid;
  Vector3d a = av.head(3);
  Vector3d x = av.tail(3);  // v
  Vector3d y = a.cross(x);  // n
  aid(3) = x.norm() / y.norm();
  x /= x.norm();
  y /= y.norm();
  Vector3d z = x.cross(y);

  Matrix3d R;
  R.col(0) = x;
  R.col(1) = y;
  R.col(2) = z;

  Vector3d aa = gc_Rodriguez(R);

  aid(0) = aa(0);
  aid(1) = aa(1);
  aid(2) = aa(2);

//  Vector4d aid;
//  Vector3d a = av.head(3);
//  Vector3d v = av.tail(3);  // v
//  Vector3d n = a.cross(v);  // n
//
//  aid(3) = v.norm() / n.norm();
//
//  Vector3d x = n / n.norm();
//  Vector3d y = v / v.norm();
//  Vector3d z = x.cross(y);
//
//  aid[0] = atan2( y(2), z(2) );
//  aid[1] = asin( - x(2) );
//  aid[2] = atan2( x(1), x(0) );

  return aid;
}

Vector6d gc_aid_to_av(Vector4d aid) {

  Vector6d av;
  Vector3d aa = aid.head(3);
  double d = 1.0 / aid(3);
  Matrix3d R = gc_Rodriguez(aa);
  av.head(3) = R.col(2) * d;
  av.tail(3) = R.col(0);

//  Vector6d av;
//  double a = aid[0];
//  double b = aid[1];
//  double g = aid[2];
//  double t = aid[3];
//
//  double s1 = sin(a);
//  double c1 = cos(a);
//  double s2 = sin(b);
//  double c2 = cos(b);
//  double s3 = sin(g);
//  double c3 = cos(g);
//
//  Matrix3d R;
//  R <<
//      c2 * c3,   s1 * s2 * c3 - c1 * s3,   c1 * s2 * c3 + s1 * s3,
//      c2 * s3,   s1 * s2 * s3 + c1 * c3,   c1 * s2 * s3 - s1 * c3,
//      -s2,                  s1 * c2,                  c1 * c2;
//
//  double d = 1.0 / t;
//  av.head(3) = -R.col(2) * d;
//  av.tail(3) = R.col(1);

  return av;
}

////////////////////////////////////////////////////////////////////////////////

Vector4d gc_av_to_asd(Vector6d av) {
  Vector4d asd;

  Vector3d a = av.head(3);
  Vector3d x = av.tail(3);  // v
  Vector3d y = a.cross(x);  // n

//  Vector2d w(y.norm(), x.norm());
//  w /= w.norm();
//  asd(3) = asin(w(1));
  
  double depth = x.norm() / y.norm();
  // double sig_d = log( (2.0*depth + 1.0) / (1.0 - 2.0*depth));

//  double quotient = depth / 0.5;
//  int integer_quotient = (int)quotient;
//  double floating_quotient = quotient - (double)integer_quotient;
//  depth = depth * floating_quotient;
//  double sig_d = log(2.0*depth + 1.0) - log(1.0 - 2.0*depth);

// double sig_d = atan(1.0 / (1.0*depth) );
// double sig_d = atan(depth);
// double sig_d = atan2(1.0, depth);
// double sig_d = atan(depth);

// double sig_d = atan2(1.0, depth) / 4.0;
  double sig_d = 1.0 / exp(-depth);
  asd(3) = sig_d;
  // cout << "sig_d = " << sig_d << endl;

  // asd(3) = depth;

  // double sig_d = tan(1.0/depth);
  // double sig_d = tan(2.0/depth);
  // double sig_d = tan(2.0*depth);
  // double sig_d = (1.0 - exp(-1.0/depth)) / (2.0*(1.0 + exp(-1.0/depth)));
  // double sig_d = (1.0 - exp(-depth)) / (2.0*(1.0 + exp(-depth)));
  // double sig_d = 1.0 / (1.0 + exp(-1.0/depth));
  // double sig_d = 1.0 / (1.0 + exp(-depth));

  x /= x.norm();
  y /= y.norm();
  Vector3d z = x.cross(y);

  Matrix3d R;
  R.col(0) = x;
  R.col(1) = y;
  R.col(2) = z;

  Vector3d aa = gc_Rodriguez(R);

  asd(0) = aa(0);
  asd(1) = aa(1);
  asd(2) = aa(2);

  return asd;
}

Vector6d gc_asd_to_av(Vector4d asd) {
  Vector6d av;

  Vector3d aa = asd.head(3);

  // double d_inv = asd(3);

  // double sig_d_inv = (1.0 - exp(-asd(3))) / (2.0 * (1.0 + exp(-asd(3))));

  // double sig_d_inv = -log(1.0/asd(3) - 1.0);
  // double sig_d_inv = log( (2.0 * asd(3) + 1.0) / (1.0 - 2.0*asd(3)) );
  // double sig_d_inv = atan(asd(3)) / 2.0;
  // double sig_d_inv = atan2(asd(3), 1.0) / 2.0;
  // double sig_d_inv = atan2(asd(3), 1.0);

  // double sig_d_inv = atan2(asd(3), 1.0) * 1.0;
  
  // double sig_d_inv = tan(4.0 * asd(3));
  double sig_d_inv = log(asd(3));
  // cout << "sig_d_inv = " << sig_d_inv << endl;

  // double sig_d_inv = cos(asd(3)) / sin(asd(3));

  // double sig_d_inv = sin(asd(3)) / cos(asd(3));
  // double sig_d_inv = sin(asd(3)) / cos(asd(3));

  Matrix3d R = gc_Rodriguez(aa);

  // av.head(3) = R.col(2) / sig_d_inv;
  av.head(3) = R.col(2) * sig_d_inv;
  av.tail(3) = R.col(0);

  return av;
}


////////////////////////////////////////////////////////////////////////////////

Vector4d gc_av_to_orth(Vector6d av) {
  Vector4d orth;

  Vector3d a = av.head(3);
  Vector3d v = av.tail(3);  // v
  Vector3d n = a.cross(v);  // n

  Vector3d x = n / n.norm();
  Vector3d y = v / v.norm();
  Vector3d z = x.cross(y);

  orth[0] = atan2( y(2), z(2) );
  orth[1] = asin( - x(2) );
  orth[2] = atan2( x(1), x(0) );

  Vector2d w( n.norm(), v.norm() );
  w = w / w.norm();

  orth[3] = asin( w(1) );

//   MatrixXd A(3,2), Q, R;
// 
//   A.col(0) = n;
//   A.col(1) = v;
// 
//   Eigen::FullPivHouseholderQR<MatrixXd> qr(A);
//   // Q = qr.householderQ();
//   Q = qr.matrixQ();
//   R = qr.matrixQR().triangularView<Upper>();
//   // std::cout << Q << "\n\n" << R << "\n\n" << Q * R - A << "\n";
// 
// //  double sigma1 = R(0,0);
// //  double sigma2 = R(1,1);
// 
// //  cout << "\ntheta from sigma1 = " << acos(sigma1) << endl;
// //  cout << "theta from sigma2 = " << asin(sigma2) << endl;
// 
// // cout << "\nsigma1 = " << sigma1<< endl;
// // cout << "sigma2 = " << sigma2<< endl;
// 
// // sigma2 /= sqrt(sigma1*sigma1 + sigma2*sigma2);
// 
//   Vector3d x = Q.col(0);
//   Vector3d y = Q.col(1);
//   Vector3d z = Q.col(2);
// 
//   orth[0] = atan2( y(2), z(2) );
//   orth[1] = asin( - x(2) );
//   orth[2] = atan2( x(1), x(0) );
//   // orth[3] = asin(sigma2);
// 
//   Vector2d w( n.norm(), v.norm() );
//   w = w / w.norm();
//   orth[3] = asin( w(1) );

  return orth;
}

Vector6d gc_orth_to_av(Vector4d auth) {
  Vector6d av;

  double a = auth[0];
  double b = auth[1];
  double g = auth[2];
  double t = auth[3];

  double s1 = sin(a);
  double c1 = cos(a);
  double s2 = sin(b);
  double c2 = cos(b);
  double s3 = sin(g);
  double c3 = cos(g);

  Matrix3d R;
  R <<
      c2 * c3,   s1 * s2 * c3 - c1 * s3,   c1 * s2 * c3 + s1 * s3,
      c2 * s3,   s1 * s2 * s3 + c1 * c3,   c1 * s2 * s3 - s1 * c3,
      -s2,                  s1 * c2,                  c1 * c2;

  double d = cos(t) / sin(t);
  av.head(3) = -R.col(2) * d;
  av.tail(3) = R.col(1);

//   Vector3d u1 = R.col(0);
//   Vector3d u2 = R.col(1);
// 
//   double sigma1 = cos(t);
//   double sigma2 = sin(t);
// 
//   double d = cos(t) / sin(t);
// 
// //  Vector3d n = sigma1 * u1;
// //  Vector3d v = sigma2 * u2;
//   Vector3d n = u1;
//   Vector3d v = u2;
//   av.head(3) = -v.cross(n) * d;
//   av.tail(3) = v;

  return av;
}

////////////////////////////////////////////////////////////////////////////////


