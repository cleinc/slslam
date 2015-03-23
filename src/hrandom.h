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

#ifndef _HRANDOM_H_
#define _HRANDOM_H_

#include <stdio.h>
#include <vector>

#define HMSG(x...)      do{ fprintf(stderr,x); fprintf(stderr,"\n"); fflush(stderr); }while(0)
#define HMSGRET(ret,arg)   do { HMSG arg; return ret; } while(0)

namespace hcv {
//-----------------------------------------------------------------------------

class HRandom
{
public:
  HRandom();
  HRandom(long seed);
  ~HRandom();

  void init(long seed);
  double random() const;                     // random double from 0 to 1
  long   uniform(long min, long max) const;   // random integer from min to max
  double normal(double m, double s) const;   // random double ~ normal(m,s)
  float  normal(float m, float s) const;

  void random(double *buf, int szbuf) const;
  void random(float *buf, int szbuf) const;
  void uniform(long min, long max, long *buf, int szbuf) const;
  void normal(double m, double s, double *buf, int szbuf) const;
  void normal(float m, float s, float *buf, int szbuf) const;
  void normal(const double *m, const double *s, double *buf, int szbuf) const;
  void normal(const float *m, const float *s, float *buf, int szbuf) const;

  bool rand_sample(int idx[/*k*/], int n, int k, bool replace=false);

  void fill_random(std::vector<double> &buf) const;
  void fill_random(std::vector<float> &buf) const;
  void fill_uniform(long min, long max, std::vector<long> &buf) const;
  void fill_normal(double m, double s, std::vector<double> &buf) const;
  void fill_normal(float m, float s, std::vector<float> &buf) const;
  bool fill_normal(const std::vector<double> &m, const std::vector<double> &s, std::vector<double> &buf) const;
  bool fill_normal(const std::vector<float> &m, const std::vector<float> &s, std::vector<float> &buf) const;

  bool rand_sample(std::vector<int> &idx, int n, int k, bool replace=false);
};

//-----------------------------------------------------------------------------
};
#endif //_HRANDOM_H_

