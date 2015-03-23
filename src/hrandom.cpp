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

#include "hrandom.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <algorithm>

static bool g_init = false;
static long int g_idum;

namespace hcv {
//-----------------------------------------------------------------------------

HRandom::HRandom()
{
  if (g_init == false) {
    g_init = true;
    g_idum = rand();
    // g_idum should be a negative integer
    if (g_idum > 0)
      g_idum = - g_idum;
    else if (g_idum == 0)
      g_idum = -1;
  }
}

HRandom::HRandom(long int seed)
{
  init(seed);
}

HRandom::~HRandom()
{
}

void HRandom::init(long int seed)
{
  g_init = true;
  g_idum = seed;
  if (g_idum > 0)
    g_idum = - g_idum;
  else if (g_idum == 0)
    g_idum = -1;
}

//-----------------------------------------------------------------------------
// Minimal random number generator of Park and Miller with Bays-Durham shuffle and
// added safeguards. returns a uniform random deviate between 0.0 and 1.0

#define IA 16807
#define IM 2147483647
#define AM (1.0/IM)
#define IQ 127773
#define IR 2836
#define NTAB 32
#define NDIV (1+(IM-1)/NTAB)
#define EPS 1.2e-7
#define RNMX (1.0-EPS)

inline long __random()
{
  int j;
  long k;
  static long iy = 0;
  static long iv[NTAB];

  if (g_idum <= 0 || !iy) {   // Initialize
    if (-(g_idum) < 1)  // Be sure to prevent g_idum == 0
      g_idum = 1;
    else 
      g_idum = -g_idum;

    for (j=NTAB+7; j>=0; j--) {   // Load the shuffle table (after 8 warm-ups)
      k = g_idum/IQ;
      g_idum = IA*(g_idum-k*IQ)-IR*k;
      if (g_idum < 0) 
        g_idum += IM;
      if (j < NTAB) 
        iv[j] = g_idum;
    }
    iy=iv[0];
  }

  k = g_idum/IQ;
  g_idum = IA*(g_idum-k*IQ)-IR*k;
  if (g_idum < 0) 
    g_idum += IM;
  j=iy/NDIV;
  iy = iv[j];
  iv[j] = g_idum;
  return iy;
}

inline double __drandom()
{
  double tmp = AM*__random();
  return (tmp > RNMX)? RNMX : tmp;
}

inline double __frandom()
{
  double tmp = AM*__random();
  return (tmp > RNMX)? RNMX : tmp;
}

//-----------------------------------------------------------------------------

double HRandom::random() const
{
  return __drandom();
}
 
long HRandom::uniform(long min, long max) const
{
  // output random integer in the interval min <= x <= max
  // multiply interval with random and truncate
  long r = long((max - min + 1) * __drandom()) + min;
  if (r > max) r = max;
  if (max < min) return 0x80000000;
  return r;
}

double HRandom::normal(double m, double s) const
{
  // normal distribution with mean m and standard deviation s
  double x1, x2, w;
  do {
    x1 = 2 * __drandom() - 1;
    x2 = 2 * __drandom() - 1;
    w = x1*x1 + x2*x2;
  } while (w >= 1 || w < 1E-30);
  w = sqrt((-2.*log(w))/w);
  x1 *= w;
  // x2 *= w;  // a second normally distributed result not used
  return x1 * s + m;
}

float HRandom::normal(float m, float s) const
{
  // normal distribution with mean m and standard deviation s
  float x1, x2, w;
  do {
    x1 = (float)(2 * __frandom() - 1);
    x2 = (float)(2 * __frandom() - 1);
    w = x1*x1 + x2*x2;
  } while (w >= 1 || w < 1E-30);
  w = (float) sqrt((-2.*log(w))/w);
  x1 *= w;
  // x2 *= w;  // a second normally distributed result not used
  return x1 * s + m;
}

//-----------------------------------------------------------------------------

void HRandom::uniform(long min, long max, long *buf, int szbuf) const
{
  for (int i = 0; i < szbuf; i++) {
    long r = long((max - min + 1) * __drandom()) + min;
    if (r > max) r = max;
    buf[i] = r;
  }
}

void HRandom::fill_uniform(long min, long max, std::vector<long> &buf) const
{
  for (size_t i = 0; i < buf.size(); i++) {
    long r = long((max - min + 1) * __drandom()) + min;
    if (r > max) r = max;
    buf[i] = r;
  }
}

void HRandom::random(double *buf, int szbuf) const
{
  for (int i = 0; i < szbuf; i++)
    buf[i] = __drandom();
}

void HRandom::fill_random(std::vector<double> &buf) const
{
  for (size_t i = 0; i < buf.size(); i++)
    buf[i] = __drandom();
}

void HRandom::random(float *buf, int szbuf) const
{
  for (int i = 0; i < szbuf; i++)
    buf[i] = (float) __frandom();
}

void HRandom::fill_random(std::vector<float> &buf) const
{
  for (size_t i = 0; i < buf.size(); i++)
    buf[i] = (float) __frandom();
}

void HRandom::normal(double m, double s, double *buf, int szbuf) const
{
  double x1, x2, w;
  for (int i = 0; i < szbuf; i++) {
    do {
      x1 = 2 * __drandom() - 1;
      x2 = 2 * __drandom() - 1;
      w = x1*x1 + x2*x2;
    } while (w >= 1 || w < 1E-30);
    w = sqrt((-2.*log(w))/w);
    buf[i] = x1 * w * s + m;
  }
}

void HRandom::fill_normal(double m, double s, std::vector<double> &buf) const
{
  double x1, x2, w;
  for (size_t i = 0; i < buf.size(); i++) {
    do {
      x1 = 2 * __drandom() - 1;
      x2 = 2 * __drandom() - 1;
      w = x1*x1 + x2*x2;
    } while (w >= 1 || w < 1E-30);
    w = sqrt((-2.*log(w))/w);
    buf[i] = x1 * w * s + m;
  }
}

bool HRandom::fill_normal(const std::vector<double> &m, const std::vector<double> &s, std::vector<double> &buf) const
{
  if (m.size() != buf.size() || s.size() != buf.size())
    HMSGRET(false, ("random: size of m,s and buf must be same (%d,%d,%d)", (int)m.size(), (int)s.size(), (int)buf.size()));

  double x1, x2, w;
  for (size_t i = 0; i < buf.size(); i++) {
    do {
      x1 = 2 * __drandom() - 1;
      x2 = 2 * __drandom() - 1;
      w = x1*x1 + x2*x2;
    } while (w >= 1 || w < 1E-30);
    w = sqrt((-2.*log(w))/w);
    buf[i] = x1 * w * s[i] + m[i];
  }
  return true;
}

void HRandom::normal(const double *m, const double *s, double *buf, int szbuf) const
{
  double x1, x2, w;
  for (int i = 0; i < szbuf; i++) {
    do {
      x1 = 2 * __drandom() - 1;
      x2 = 2 * __drandom() - 1;
      w = x1*x1 + x2*x2;
    } while (w >= 1 || w < 1E-30);
    w = sqrt((-2.*log(w))/w);
    buf[i] = x1 * w * s[i] + m[i];
  }
}

void HRandom::normal(float m, float s, float *buf, int szbuf) const
{
  float x1, x2, w;
  for (int i = 0; i < szbuf; i++) {
    do {
      x1 = (float)(2 * __frandom() - 1);
      x2 = (float)(2 * __frandom() - 1);
      w = x1*x1 + x2*x2;
    } while (w >= 1 || w < 1E-30);
    w = (float) sqrt((-2.*log(w))/w);
    buf[i] = x1 * w * s + m;
  }
}

void HRandom::fill_normal(float m, float s, std::vector<float> &buf) const
{
  float x1, x2, w;
  for (size_t i = 0; i < buf.size(); i++) {
    do {
      x1 = (float)(2 * __frandom() - 1);
      x2 = (float)(2 * __frandom() - 1);
      w = x1*x1 + x2*x2;
    } while (w >= 1 || w < 1E-30);
    w = (float) sqrt((-2.*log(w))/w);
    buf[i] = x1 * w * s + m;
  }
}

void HRandom::normal(const float *m, const float *s, float *buf, int szbuf) const
{
  float x1, x2, w;
  for (int i = 0; i < szbuf; i++) {
    do {
      x1 = (float)(2 * __frandom() - 1);
      x2 = (float)(2 * __frandom() - 1);
      w = x1*x1 + x2*x2;
    } while (w >= 1 || w < 1E-30);
    w = (float) sqrt((-2.*log(w))/w);
    buf[i] = x1 * w * s[i] + m[i];
  }
}

bool HRandom::fill_normal(const std::vector<float> &m, const std::vector<float> &s, std::vector<float> &buf) const
{
  if (m.size() != buf.size() || s.size() != buf.size())
    HMSGRET(false, ("random: size of m,s and buf must be same (%d,%d,%d)", (int)m.size(), (int)s.size(), (int)buf.size()));

  float x1, x2, w;
  for (size_t i = 0; i < buf.size(); i++) {
    do {
      x1 = (float)(2 * __frandom() - 1);
      x2 = (float)(2 * __frandom() - 1);
      w = x1*x1 + x2*x2;
    } while (w >= 1 || w < 1E-30);
    w = (float) sqrt((-2.*log(w))/w);
    buf[i] = x1 * w * s[i] + m[i];
  }
  return true;
}


//-----------------------------------------------------------------------------

struct _rand_sample_t { double v; int i; };

int _rand_sample_cmp(const void *p0, const void *p1)
{
  struct _rand_sample_t *s0 = (struct _rand_sample_t*) p0;
  struct _rand_sample_t *s1 = (struct _rand_sample_t*) p1;
  return s0->v < s1->v? -1 : s0->v > s1->v? 1 : 0;
}

bool HRandom::rand_sample(int idx[/*k*/], int n, int k, bool replace/*=false*/)
{
  if (replace == true) {
    for (int i=0; i<k; i++)
      idx[i] = (int)(n*random());
  }
  else if (n < k)
    HMSGRET(false, ("random: k must be <= n with replace (%d:%d)", k,n));
  else
    if (4*k > n) {
      struct _rand_sample_t *arr = new struct _rand_sample_t[n];
      for (int i=0; i<n; i++)
        arr[i].v = random(), arr[i].i = i;
      qsort(arr, n, sizeof(struct _rand_sample_t), _rand_sample_cmp);
      for (int i=0; i<k; i++)
        idx[i] = arr[i].i;
      delete[] arr;
    }
    else {
      char *flag = new char[n];
      memset(flag, 0, sizeof(char)*n);
      for (int i=0, j; i<k; i++) {
        for (int l=0; flag[j = (int)(n*random())] > 0 && l<1000; ++l) continue;
        if (flag[j] > 0)  for (j=0; flag[j]>0; ++j) continue;
        idx[i] = j, flag[j] = 1;
      }
      delete[] flag;
    }
  return true;
}


bool HRandom::rand_sample(std::vector<int> &idx, int n, int k, bool replace)
{
  idx.resize(k);
  if (replace == true)
    for (int i=0; i<k; i++)
      idx[i] = (int)(n*random());
  else if (n < k)
    HMSGRET(false, ("random: k must be <= n with replace (%d:%d)", k,n));
  else if (4*k > n) {
    std::vector< std::pair<double,int> > arr(n);
    for (int i=0; i<n; i++)
      arr[i].first = random(), arr[i].second = i;
    std::sort(arr.begin(), arr.end());
    for (int i=0; i<k; i++)
      idx[i] = arr[i].second;
  }
  else {
    std::vector<bool> flag(n,false);
    for (int i=0, j; i<k; i++) {
//      while (flag[j = (int)(n*random())] == true)  continue;
      for (int l=0; flag[j=(int)(n*random())]==true && l<1000; ++l) continue;
      if (flag[j] == true)  for (j=0; flag[j]==true; ++j) continue;
      idx[i] = j, flag[j] = true;
    }
  }
  return true;
}


//-----------------------------------------------------------------------------
};

