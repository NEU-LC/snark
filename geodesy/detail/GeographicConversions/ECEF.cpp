// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2011 The University of Sydney
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of the University of Sydney nor the
//    names of its contributors may be used to endorse or promote products
//    derived from this software without specific prior written permission.
//
// NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
// GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
// HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS OR IMPLIED
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
// IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/// @author andrew maclean

#include <cmath>

#include "ECEF.h"

using namespace std;

namespace snark { namespace detail { namespace GeographicGeodeticRectangular {

void CGeodeticECEF::toECEF ( double const & latitude, double const & longitude, double const & height, 
  double & X, double & Y, double & Z)
{

  double lat = dtr * latitude;
  double slat = sin(lat);
  double clat = cos(lat);
  double lng = dtr * longitude;
  double clng = cos(lng);
  double slng = sin(lng);

  // Radius of curvature in the prime vertical.
  double N = E.SemimajorAxis()/sqrt(1.0-E.EccentricitySq()*slat*slat);

  X = (N+height)*clat*clng;
  Y = (N+height)*clat*slng;
  Z = (N*(1-E.EccentricitySq())+height)*slat;
}

void CGeodeticECEF::fromECEF_Inexact ( double const & X, double const & Y, double const & Z, 
  double & latitude, double & longitude, double & height)
{
  double a = E.SemimajorAxis();
  double b = E.SemiminorAxis();
  double p = sqrt(X*X+Y*Y);

  // Here we are somewhere on a line netween the North or South pole. 
  if ( p == 0 )
  {
    if ( Z < 0 )
    {
      latitude = -90;
      height = -Z - b;
    }
    else
    {
      latitude = 90;
      height = Z - b;
    }
    longitude = 0;
    return ;
  }

  // Here we are somewhere on the equator. 
  if ( Z == 0 )
  {
    latitude = 0;
    longitude = atan2(Y,X)*rtd;
    height = p - a;
    return ;
  }


  double t = atan(Z*a/(p*b));
  double st = sin(t);
  double ct = cos(t);

  latitude = atan2((p-E.EccentricitySq()*a*ct*ct*ct),(Z + E.e_dash_sq()*b*st*st*st));
  latitude = -(latitude - pi2);

  double slat = sin(latitude);
  double clat = cos(latitude);
  // Radius of curvature in the prime vertical.
  double Rn = a/sqrt(1.0-E.EccentricitySq()*slat*slat);

  height = p/clat - Rn;
  latitude = latitude * rtd;
  longitude = atan2(Y,X)*rtd;

}


void CGeodeticECEF::fromECEF ( double const & X, double const & Y, double const & Z, 
  double & latitude, double & longitude, double & height)
{
  double a = E.SemimajorAxis();
  double b = E.SemiminorAxis();
  double r = sqrt(X*X+Y*Y);
  
  // Here we are somewhere on a line netween the North or South pole. 
  if ( r == 0 )
  {
    if ( Z < 0 )
    {
      latitude = -90;
      height = -Z - b;
    }
    else
    {
      latitude = 90;
      height = Z - b;
    }
    longitude = 0;
    return ;
  }

  // Here we are somewhere on the equator. 
  if ( Z == 0 )
  {
    latitude = 0;
    longitude = atan2(Y,X)*rtd;
    height = r - a;
    return ;
  }

  // Adjust for the southern hemisphere.
  if ( Z < 0 ) b = -b;

  double E = (b*Z-(a*a-b*b))/(a*r);
  double F = (b*Z+(a*a-b*b))/(a*r);
  double P = (E*F+1.0)*4.0/3.0;
  double Q = (E*E-F*F)*2.0;
  double D = P*P*P+Q*Q;
  double v;
  if ( D >= 0 )
  {
    double s1 = sqrt(D)-Q;
    if ( s1 > 0 )
    {
      s1 = pow(std::abs(s1),1.0/3.0);
    }
    else
    {
      s1 = -pow(std::abs(s1),1.0/3.0);
    }
    double s2 = sqrt(D)+Q;
    if ( s2 > 0 )
    {
      s2 = pow(s2,1.0/3.0);
    }
    else
    {
      s2 = -pow(std::abs(s2),1.0/3.0);
    }
    v = s1-s2;
    // This improves the accuracy of the solution.
    // The following equation is more accurate than v (above) 
    // if v^2 < P, which is the case everywhere outside the radial 
    // distance from the Earth's centre of about 70km.
    // if ( v * v < abs(P) )
      v = -(v*v*v+2.0*Q)/(3.0*P);
  }
  else
  {
    v = 2.0 * sqrt(-P)*cos(acos(Q/P/sqrt(-(P)))/3.0);
  }
  double G = 0.5*(E+sqrt(E*E+v));
  double t = sqrt(G*G+(F-v*G)/(G+G-E))-G;
  double fi = atan((1-t*t)*a/(2.0*b*t));
  height = (r-a*t)*cos(fi)+(Z-b)*sin(fi);
  latitude = fi * rtd;
  longitude = atan2(Y,X)*rtd;

}

} } } // namespace snark { namespace detail { namespace GeographicGeodeticRectangular {
