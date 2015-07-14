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

#if !defined(ECEF_H)
#define ECEF_H

#include "Ellipsoid.h"


namespace GeographicGeodeticRectangular
{

  //! This class converts between geodetic and ECEF rectangular coordinates on the ellipsoid.
  /*!
  CLASS
    CGeodeticECEF

    When using this class, you must instantiate it with an instance of the ellipsoid.

  \version 1.0
  first version
  
  \date 16-Sept-2003
  
  \author Andrew Maclean

  \par license
  This software is distributed WITHOUT ANY WARRANTY; without even the 
  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
  YOU, as the USER, take SOLE RESPONSIBILITY for, and are responsible for 
  determining the FITNESS and APPLICABILITY of this software for a particular 
  purpose. The author or any other person or entity are not liable in any way 
  for the use or misuse of any program or software using this software. 
  
  \todo

  \bug

  */
  class CGeodeticECEF
  {
  public:
    CGeodeticECEF( Ellipsoid::CEllipsoid Ellipsoid ) :
      rtd(45.0/std::atan(1.0))
      , dtr(std::atan(1.0)/45.0)
      , pi2(2.0*std::atan(1.0))
    { 
      E = Ellipsoid; 
    };
    virtual ~CGeodeticECEF(void){};

    //! Assignment
    CGeodeticECEF &operator = ( CGeodeticECEF const & rhs )
    {
      if ( this == &rhs )
        return *this;

      this->E = rhs.E;

      return *this;
    }

  public:
    //! Convert geodetic coordinates to Cartesian coordinates.
    /*!
    
      Eastern Longitudes = Positive Y
      Northern Latitudes = Positive Z
      
      Input:
      Geodetic latitude, longitude and height above the ellipsoid.

      @param latitude - in degrees
      @param longitude - in degrees
      @param height - in the same units as the ellipsoid ( usually meters)

      Output:
      Earth Centered Earth Fixed Cartesian Coordinates.

      @param X
      @param Y
      @param Z
    */
    void toECEF ( double const & latitude, double const & longitude, double const & height, 
      double & X, double & Y, double & Z); 

    //! Convert Cartesian coordinates to geodetic coordinates.
    /*!
      This conversion is not exact but does provide centimeter accuracy for heights <= 1000km
      (Bowring B. 1976. "Transformation from spatial to geographic coordinates.", Survey Review, XXII pp323-327 )

      Eastern Longitudes = Positive Y
      Northern Latitudes = Positive Z

      Input:
      Earth Centered Earth Fixed Cartesian Coordinates.

      @param X
      @param Y
      @param Z

      Output:
      Geodetic latitude, longitude and height above the ellipsoid.

      @param latitude - in degrees
      @param longitude - in degrees
      @param height - in the same units as the ellipsoid ( usually meters)

    */
    void fromECEF_Inexact ( double const & X, double const & Y, double const & Z, 
      double & latitude, double & longitude, double & height); 

    //! Convert Cartesian coordinates to geodetic coordinates - exact solution.
    /*!
      This is an implementation of closed-form solution published by K. M. Borkowski.

      Eastern Longitudes = Positive Y
      Northern Latitudes = Positive Z
      
      Input:
      Earth Centered Earth Fixed Cartesian Coordinates.

      @param X
      @param Y
      @param Z

      Output:
      Geodetic latitude, longitude and height above the ellipsoid.

      @param latitude - in degrees
      @param longitude - in degrees
      @param height - in the same units as the ellipsoid ( usually meters)

      @return false if Z==0 or (X*X+Y*Y) == 0, true otherwise. A false return 
      indicates that the calculation was not performed.

      The best closed-form solution seems  to be that of K. M. Borkowski, 
      a Polish radio-astronomer. It requires  finding a root of a 
      fourth-degree polynomial, and is therefore  possible using the 
      classical fourth-degree formula for roots. The algorithm is given 
      in detail in the (new) Explanatory Supplement for 
      the Astronomical Almanac (P. Seidelmann, editor). 
      
      There's a link to an abstract of Borkowski's paper on his 
      personal Web page:

        http://www.astro.uni.torun.pl/~kb/personal.html   

      The abstract is at

        http://www.astro.uni.torun.pl/~kb/abstract.html#Transf2   


    */
    void fromECEF ( double const & X, double const & Y, double const & Z, 
      double & latitude, double & longitude, double & height);

  private:
    Ellipsoid::CEllipsoid E;
    //! Degrees to radians.
    double const rtd;
    //! Radians to degrees.
    double const dtr;
    //! pi/2
    double const pi2;
  };

}

#endif // ECEF_H
