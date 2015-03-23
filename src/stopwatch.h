// This file is part of VisionTools.
//
// Copyright 2011 Hauke Strasdat (Imperial College London)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights  to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.

#ifndef VISIONTOOLS_STOP_WATCH_H
#define VISIONTOOLS_STOP_WATCH_H

#include <cassert>
#include <sys/time.h>
#include <stdio.h>
#include <unistd.h>

#include <stdexcept>


//namespace VisionTools
//{

typedef struct _time_statistics {
  int c;
  double t;
} time_statistics;

class   StopWatch
{
public:
  StopWatch()
  {
    running_ = false;
    time_ = 0;
    prev_msec_ = 0;

    p1c_ = 0;
    p2c_ = 0;
    p3c_ = 0;
    p4c_ = 0;
    p5c_ = 0;

    p1t_ = 0.0;
    p2t_ = 0.0;
    p3t_ = 0.0;
    p4t_ = 0.0;
    p5t_ = 0.0;
  }

  void start()
  {
    assert(running_==false);
    gettimeofday(&start_time_, NULL);
    running_ = true;
  }

  void stop()
  {
    assert(running_);
    gettimeofday(&end_time_, NULL);
    long seconds  = end_time_.tv_sec  - start_time_.tv_sec;
    long useconds = end_time_.tv_usec - start_time_.tv_usec;
    time_ = ((seconds) + useconds*0.000001);
    running_ = false;
  }

  double read_current_time()
  {
    assert(running_);
    timeval cur_time;
    gettimeofday(&cur_time, NULL);
    long seconds  = cur_time.tv_sec  - start_time_.tv_sec;
    long useconds = cur_time.tv_usec - start_time_.tv_usec;
    return ((seconds) + useconds*0.000001);
  }

  double get_stopped_time()
  {
    assert(running_==false);
    return time_;
  }

  inline void reset()
  {
    time_ = 0;
  }

  double elapse_msec()
  {
    double curr_msec = read_current_time() * 1000;
    double rt_msec = curr_msec - prev_msec_;
    prev_msec_ = curr_msec;

    return rt_msec;
  }

  void tick()
  {
    prev_msec_ = read_current_time();
  }

  double tock()
  {
    return read_current_time() - prev_msec_;
  }

  void proc_1_tick() { p1c_++; tick(); }
  void proc_2_tick() { p2c_++; tick(); }
  void proc_3_tick() { p3c_++; tick(); }
  void proc_4_tick() { p4c_++; tick(); }
  void proc_5_tick() { p5c_++; tick(); }

  void proc_1_tock() { p1t_ += tock(); }
  void proc_2_tock() { p2t_ += tock(); }
  void proc_3_tock() { p3t_ += tock(); }
  void proc_4_tock() { p4t_ += tock(); }
  void proc_5_tock() { p5t_ += tock(); }

  void proc_1_statistics( time_statistics & ts ) { ts.c = p1c_; ts.t = p1t_; }
  void proc_2_statistics( time_statistics & ts ) { ts.c = p2c_; ts.t = p2t_; }
  void proc_3_statistics( time_statistics & ts ) { ts.c = p3c_; ts.t = p3t_; }
  void proc_4_statistics( time_statistics & ts ) { ts.c = p4c_; ts.t = p4t_; }
  void proc_5_statistics( time_statistics & ts ) { ts.c = p5c_; ts.t = p5t_; }

private:
  int p1c_;
  int p2c_;
  int p3c_;
  int p4c_;
  int p5c_;

  double p1t_;
  double p2t_;
  double p3t_;
  double p4t_;
  double p5t_;

  timeval start_time_;
  timeval end_time_;
  double time_;
  bool running_;
  double prev_msec_;
};
//}

#endif
