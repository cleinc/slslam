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

#include "cplot.h"
#include "gc.h"
#include "parameter.h"

#include "gflags/gflags.h"

DECLARE_int32(delay);

cv::Mat image;
cv::Mat img_glscene;
cv::Mat img_linetracking;
cv::Mat img_all_view;

void gluPerspective(double fovy,double aspect, double zNear, double zFar)
{
 // Start in projection mode.
 glMatrixMode(GL_PROJECTION);
 glLoadIdentity();
 double xmin, xmax, ymin, ymax;
 ymax = zNear * tan(fovy * M_PI / 360.0);
 ymin = -ymax;
 xmin = ymin * aspect;
 xmax = ymax * aspect;
 glFrustum(xmin, xmax, ymin, ymax, zNear, zFar);
}

CPlot::CPlot() {
  debug_plot = false;
  save_image = true;

  gl_win_width = image_width;
  gl_win_height = image_height;

  frame_id = 0;

  if (!glfwInit())
      exit(EXIT_FAILURE);

  window = glfwCreateWindow(gl_win_width, gl_win_height,
      "Trajectory and Landmarks", NULL, NULL);
  if (!window)
  {
      glfwTerminate();
      exit(EXIT_FAILURE);
  }

  glfwMakeContextCurrent(window);
  // glfwSetWindowPos(window, 1200, 400);
  glfwSetWindowPos(window, 0, 0);

  glViewport(0, 0, gl_win_width, gl_win_height);
  glMatrixMode( GL_PROJECTION);
  glLoadIdentity ();
  glEnable( GL_DEPTH_TEST);
  gluPerspective(45, (float) gl_win_width / gl_win_height, .1, 150);

  glMatrixMode (GL_MODELVIEW);
  glClearColor( 1.0f, 1.0f, 1.0f, 1.0f );

  glPrepare( Vector3d( 0, 0, 0 ) );
  drawFloorGrid();
  drawRobot( pose_t() );
  glfwSwapBuffers(window);
}

CPlot::~CPlot() {
  glfwTerminate();
//  cv::destroyAllWindows();
}

void CPlot::drawResult( vector<pose_t> trajectory, vector<Vector6d> lines ) {

  pose_t T  = trajectory.back();

  glPrepare( T.t );
  drawLine( lines );
  drawFloorGrid();
  drawTrajectory( trajectory );
  drawDropLine( trajectory );
  drawRobot( T );

  glfwSwapBuffers(window);
  // saveGlScene( frame_id );

#ifdef DRAW_ALL_VIEW
  // saveAllView();
#endif
}

void CPlot::saveAllView() {
  char name[256];
  sprintf( name, "../output/allview/%05d.jpg", frame_id );

  int shrink_width = 384;
  int shrink_height = 288;

  img_all_view  = cv::Mat( image_height, image_width + shrink_width, CV_8UC3,
      cv::Scalar( 255, 255, 255 ) );

  cv::Mat roi;
  cv::Rect r( 50, 40, 540, 405 );
  roi = img_all_view( cv::Rect( 0, 0, shrink_width, shrink_height ) );
  resize( img_linetracking(r), roi, roi.size(), 0, 0 );
  roi = img_all_view( cv::Rect( shrink_width, 0, image_width, image_height ) );
  img_glscene.copyTo( roi );

  // cv::imwrite( name, img_all_view );
}

void CPlot::drawResultTrajectory( vector<pose_t> trajectory ) {
  pose_t T  = trajectory.back();

  glPrepare( Vector3d( 0, 0, 0 ) );
  drawFloorGrid();
  drawTrajectory( trajectory );
  drawDropLine( trajectory );
  drawRobot( T );

  glfwSwapBuffers(window);

  // saveGlScene( frame_id );
}

void CPlot::glPrepare( Eigen::Vector3d twc )
{
  float zoom = 15.0f;

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

  glRotatef(180.0f, 0, 0, 1);
  glRotatef(-42.0f, 1, 0, 0);
  glRotatef(-45.0f, 0, 1, 0);

  glTranslatef(-zoom - twc(0), zoom - twc(1), -zoom - twc(2));
  // glTranslatef(-3.0f-twc(0), 7.0f - twc(1), -15 - twc(2));
}

void CPlot::drawFloorGrid()
{
  GLfloat x, z;

  // glColor3f( 0.95f, 0.95f, 0.95f );
  glColor3f( 0.8f, 0.8f, 0.8f );

  glBegin( GL_LINES );

  GLfloat L = 74.99f;
  for ( x = - L; x < L; x += 1.0f )
  {
    glVertex3f( x, 1.5f,  L );
    glVertex3f( x, 1.5f, -L );
  }

  for ( z = - L; z < L; z += 1.0f )
  {
    glVertex3f(  L, 1.5f, z );
    glVertex3f( -L, 1.5f, z );
  }

  glEnd();
}

void CPlot::drawTrajectory( vector<pose_t> trajectory ) {
  glColor3f( 0.0f, 0.0f, 1.0f );
  glBegin( GL_LINES );

  Vector3d t1( 0, 0, 0 );

  for ( size_t i = 1; i < trajectory.size(); ++i ) {
    Vector3d t2 = trajectory[i].t;

    glVertex3f( t1(0), t1(1), t1(2) );
    glVertex3f( t2(0), t2(1), t2(2) );

    t1 = t2;
  }

  glEnd();
}

void CPlot::drawDropLine( vector<pose_t> trajectory ){
  glColor3f( 0.5f, 0.5f, 0.5f );
  glBegin( GL_LINES );

  for ( size_t i = 1; i < trajectory.size(); ++i ) {
    if ( i % 3 != 0 )
      continue;

    Vector3d t1 = trajectory[i].t;

    glVertex3f( t1(0), t1(1), t1(2) );
    glVertex3f( t1(0),     0, t1(2) );
  }

  glEnd();
}

void CPlot::drawRobot( pose_t T )
{

  float x_cam = T.t(0);
  float y_cam = T.t(1);
  float z_cam = T.t(2);

  glColor3f( 1.0f, 0.0f, 0.0f );
  glBegin( GL_TRIANGLES );

  Vector3d cn1(   0.3, 0, - 0.3 );  // cn1 - cam_node_1
  Vector3d cn2( - 0.3, 0, - 0.3 );
  Vector3d cn3(     0, 0,   0.8 );

  double factor = 1.0;
  cn1 = T.R * cn1 * factor;
  cn2 = T.R * cn2 * factor;
  cn3 = T.R * cn3 * factor;

  glVertex3f( x_cam + cn1(0), y_cam + cn1(1), z_cam + cn1(2) );
  glVertex3f( x_cam + cn2(0), y_cam + cn2(1), z_cam + cn2(2) );
  glVertex3f( x_cam + cn3(0), y_cam + cn3(1), z_cam + cn3(2) );

  glEnd();
}

void CPlot::drawLine( vector<Vector6d> lines ) {
  glColor3f( 0.0f, 0.7f, 0.0f );
  glBegin( GL_LINES );

  // cout << "lines.size() = " << lines.size() << endl;
  for ( size_t i = 0; i < lines.size(); ++i ) {
    Vector6d line = lines[i];

    glVertex3f( line(0), line(1), line(2) );
    glVertex3f( line(3), line(4), line(5) );
  }

  glEnd();
}

void CPlot::drawObservation(
    const vector<int> &segids0, const vector<int> &segids1,
    const vector<Vector8d> &ft0, const vector<Vector8d> &ft1 )
{
  int imw = image_width;
  int imh = image_height;
  int size = min(ft0.size(), ft1.size());
  image  = cv::Mat::zeros( imh, imw * 2 + 1, CV_8UC3 );
  char name[256];
  sprintf( name, "%s/%04d.jpg", img_dir.c_str(), frame_id );
  cv::Mat left_img = cv::imread(name, CV_LOAD_IMAGE_COLOR);
  cv::Mat spacemat = cv::Mat(left_img.rows, 1, CV_8UC3);
  cv::hconcat(left_img, spacemat, left_img);
  sprintf( name, "%s/../right_rect/%04d.jpg", img_dir.c_str(), frame_id );
  cv::Mat right_img = cv::imread(name, CV_LOAD_IMAGE_COLOR);
  cv::Mat stereo_img;
  cv::hconcat(left_img, right_img, stereo_img);
  image = stereo_img.clone();

  cv::Point2f p1, p2, p3, p4;
  p1.x = imw;
  p1.y = 0;
  p2.x = imw;
  p2.y = imh - 1;
  cv::line( image, p1, p2, cvScalar( 255, 255, 255 ) );

  char tid[16];
  int font = CV_FONT_HERSHEY_PLAIN;
  double font_scale = 0.8;

  for ( int i = 0; i < size; ++i ) {
//    p1.x = (int) ( ft0[i](0) * focal_length + cx1 );
//    p1.y = (int) ( ft0[i](1) * focal_length + cy1 );
//    p2.x = (int) ( ft0[i](2) * focal_length + cx1 );
//    p2.y = (int) ( ft0[i](3) * focal_length + cy1 );
//    cv::line( image, p1, p2, cvScalar( 255, 255, 255 ) );

//    sprintf( tid, "%d", ft0[i].id );
//    cv::putText( image, tid, p1, font, font_scale, CV_RGB(255,255,255) );

    cv::Scalar hashcolor = cv::Scalar(123*segids1[i]%255, 234*segids1[i]%255,
        345*segids1[i]%255);
    p1.x = (float) ( ft1[i](0) * focal_length + cx1 );
    p1.y = (float) ( ft1[i](1) * focal_length + cy1 );
    p2.x = (float) ( ft1[i](2) * focal_length + cx1 );
    p2.y = (float) ( ft1[i](3) * focal_length + cy1 );
    cv::line( image, p1, p2, hashcolor );

    sprintf( tid, "%d", segids1[i] );
    // cv::putText( image, tid, p2, font, font_scale, hashcolor );

//    p1.x = (int) ( ft0[i](4) * focal_length + cx1 ) + imw + 1;
//    p1.y = (int) ( ft0[i](5) * focal_length + cy1 );
//    p2.x = (int) ( ft0[i](6) * focal_length + cx1 ) + imw + 1;
//    p2.y = (int) ( ft0[i](7) * focal_length + cy1 );
//    cv::line( image, p1, p2, cvScalar( 255, 255, 255 ) );

    // sprintf( tid, "%d", segids0[i] );
    // cv::putText( image, tid, p1, font, font_scale, CV_RGB(255,255,255) );

    p3.x = (float) ( ft1[i](4) * focal_length + cx1 ) + imw + 1;
    p3.y = (float) ( ft1[i](5) * focal_length + cy1 );
    p4.x = (float) ( ft1[i](6) * focal_length + cx1 ) + imw + 1;
    p4.y = (float) ( ft1[i](7) * focal_length + cy1 );
    cv::line( image, p3, p4, hashcolor );

    // sprintf( tid, "%d", segids1[i]);
    // cv::putText( image, tid, p4, font, font_scale, hashcolor );
    cv::imshow( "Observations", image );
    static bool is_first = true;
    if(is_first == true) {
      // cv::moveWindow("Observations", 1200, 900);
      cv::moveWindow("Observations", 0, 560);
      is_first = false;
    }
//    printf("%s: %g %g %g %g %g %g %g %g\n", tid, p1.x, p1.y, p2.x, p2.y, p3.x,
//        p3.y, p4.x, p4.y);
  }
  cv::waitKey(FLAGS_delay);
  // saveTrackingView( frame_id );
}

void CPlot::saveTrackingView( int i ) {
  if ( save_image == false )
    return;

  char name[256];

  sprintf( name, "output/tracking/%04d.jpg", i );
  // cv::imwrite( name, image );
}

void CPlot::saveGlScene( int idx ) {
  if ( save_image == false )
    return;

  unsigned char* imageData =
      ( unsigned char * )malloc( gl_win_width * gl_win_height * 3 );
  glReadPixels( 0, 0, gl_win_width-1, gl_win_height-1,
      GL_RGB, GL_UNSIGNED_BYTE, imageData );

  cv::Mat img0( gl_win_height, gl_win_width, CV_8UC3, imageData );
  flip( img0, img0, 0 );
  cvtColor( img0, img0, CV_BGR2RGB );

  char im_name[256];
  sprintf( im_name, "../output/glscene/%05d.png", idx );
  // cv::imwrite( im_name, img0 );

  img0.copyTo( img_glscene );
//  img_glscene = img0;  <--- do not work normally

  free(imageData);
}

void CPlot::setDebugPlot( bool v ) {
  debug_plot = v;
}

void plotDrawTracking( Vector8d obs, int line_id, int frame_id ) {
/*
  char name[256];
  cv::Mat image  = cv::Mat::zeros( image_height, image_width * 2, CV_8UC3 );

  sprintf( name, "/home/slam/DATA/stereo_image/left_rect/%04d.jpg", frame_id );
  cv::Mat img = cv::imread( name, CV_LOAD_IMAGE_COLOR );
  img.copyTo( image( cv::Rect( 0, 0, img.cols, img.rows ) ) );

  sprintf( name, "/home/slam/DATA/stereo_image/right_rect/%04d.jpg", frame_id );
  img = cv::imread( name, CV_LOAD_IMAGE_COLOR );
  img.copyTo( image( cv::Rect( image_width, 0, img.cols, img.rows ) ) );

  cv::Point p1, p2;
  p1.x = (int) ( obs(0) * focal_length + cx1 );
  p1.y = (int) ( obs(1) * focal_length + cy1 );
  p2.x = (int) ( obs(2) * focal_length + cx1 );
  p2.y = (int) ( obs(3) * focal_length + cy1 );
  cv::line( image, p1, p2, cvScalar( 0, 0, 255 ) );

  p1.x = (int) ( obs(4) * focal_length + cx1 + image_width );
  p1.y = (int) ( obs(5) * focal_length + cy1 );
  p2.x = (int) ( obs(6) * focal_length + cx1 + image_width );
  p2.y = (int) ( obs(7) * focal_length + cy1 );
  cv::line( image, p1, p2, cvScalar( 0, 0, 255 ) );

  sprintf( name, "%d-%d", line_id, frame_id );
  cv::namedWindow( name, cv::WINDOW_AUTOSIZE );
  cv::imshow( name, image );

  sprintf( name, "output/line_tracking/%d-%d.png", line_id, frame_id );
  cv::imwrite( name, image );

  cv::waitKey(1);
  cv::destroyWindow( name );
*/
}

void CPlot::drawImageTracking( vector<Vector8d> obs ) {
  char name[256];
  sprintf( name, "%s/%04d.jpg", img_dir.c_str(), frame_id );

  img_linetracking = cv::imread( name, CV_LOAD_IMAGE_COLOR );

  for ( size_t i = 0; i < obs.size(); ++i ) {
    Vector8d endpoints = obs[i];

    cv::Point p1, p2;
    p1.x = (int) ( endpoints[0] * focal_length + cx1 );
    p1.y = (int) ( endpoints[1] * focal_length + cy1 );
    p2.x = (int) ( endpoints[2] * focal_length + cx1 );
    p2.y = (int) ( endpoints[3] * focal_length + cy1 );
    cv::line( img_linetracking, p1, p2, cvScalar( 0, 255, 0 ) );
  }
}
