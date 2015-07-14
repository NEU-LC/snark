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

#if !defined(ELLIPSOID_H)
#define ELLIPSOID_H

//! Ellipsoids representing the shape of the earth.
namespace Ellipsoid {

  //! Stores constants for the ellipsoid.
  /*!
  CLASS
    CEllipsoid

    The parameters must be set via the constructor.

    The available ellipsoids are:

      <div align="center">
      <center>
      <table border="1" cellpadding="0" cellspacing="0" style="border-collapse: collapse" bordercolor="#111111" id="AutoNumber1">
        <tr>
          <td width="213"><b>Ellipsoid</b></td>
          <td width="151"><b>Semi-major axis</b></td>
          <td width="127"><b>1/flattening </b></td>
        </tr>
        <tr>
          <td width="213">Airy 1830</td>
          <td width="151">6377563.396</td>
          <td width="127">299.3249646</td>
        </tr>
        <tr>
          <td width="213">Modified Airy</td>
          <td width="151">6377340.189</td>
          <td width="127">299.3249646</td>
        </tr>
        <tr>
          <td width="213">Australian National</td>
          <td width="151">6378160</td>
          <td width="127">298.25</td>
        </tr>
        <tr>
          <td width="213">Bessel 1841 (Namibia)</td>
          <td width="151">6377483.865</td>
          <td width="127">299.1528128</td>
        </tr>
        <tr>
          <td width="213">Bessel 1841</td>
          <td width="151">6377397.155</td>
          <td width="127">299.1528128</td>
        </tr>
        <tr>
          <td width="213">Clarke 1866</td>
          <td width="151">6378206.4</td>
          <td width="127">294.9786982</td>
        </tr>
        <tr>
          <td width="213">Clarke 1880</td>
          <td width="151">6378249.145</td>
          <td width="127">293.465</td>
        </tr>
        <tr>
          <td width="213">Everest (India 1830)</td>
          <td width="151">6377276.345</td>
          <td width="127">300.8017</td>
        </tr>
        <tr>
          <td width="213">Everest (Sabah Sarawak)</td>
          <td width="151">6377298.556</td>
          <td width="127">300.8017</td>
        </tr>
        <tr>
          <td width="213">Everest (India 1956)</td>
          <td width="151">6377301.243</td>
          <td width="127">300.8017</td>
        </tr>
        <tr>
          <td width="213">Everest (Malaysia 1969)</td>
          <td width="151">6377295.664</td>
          <td width="127">300.8017</td>
        </tr>
        <tr>
          <td width="213">Everest (Malay. &amp; Sing)</td>
          <td width="151">6377304.063</td>
          <td width="127">300.8017</td>
        </tr>
        <tr>
          <td width="213">Everest (Pakistan)</td>
          <td width="151">6377309.613</td>
          <td width="127">300.8017</td>
        </tr>
        <tr>
          <td width="213">Modified Fischer 1960</td>
          <td width="151">6378155</td>
          <td width="127">298.3</td>
        </tr>
        <tr>
          <td width="213">Helmert 1906</td>
          <td width="151">6378200</td>
          <td width="127">298.3</td>
        </tr>
        <tr>
          <td width="213">Hough 1960</td>
          <td width="151">6378270</td>
          <td width="127">297</td>
        </tr>
        <tr>
          <td width="213">Indonesian 1974</td>
          <td width="151">6378160</td>
          <td width="127">298.247</td>
        </tr>
        <tr>
          <td width="213">International 1924</td>
          <td width="151">6378388</td>
          <td width="127">297</td>
        </tr>
        <tr>
          <td width="213">Krassovsky 1940</td>
          <td width="151">6378245</td>
          <td width="127">298.3</td>
        </tr>
        <tr>
          <td width="213">GRS 80</td>
          <td width="151">6378137</td>
          <td width="127">298.257222101</td>
        </tr>
        <tr>
          <td width="213">South American 1969</td>
          <td width="151">6378160</td>
          <td width="127">298.25</td>
        </tr>
        <tr>
          <td width="213">WGS 72</td>
          <td width="151">6378135</td>
          <td width="127">298.26</td>
        </tr>
        <tr>
          <td width="213">WGS 84</td>
          <td width="151">6378137</td>
          <td width="127">298.257223563</td>
        </tr>
      </table>
      </center>
      </div>

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
  class CEllipsoid
  {
  public:
    //! Construct an ellipsoid with a given semi major axis and inverse flattening.
    /*!
      @param semimajorAxis - semi-major axis in meters.
      @param inverseFlattening - inverse flatteneing.
    */
    CEllipsoid ( double const & semimajorAxis = 0, double const & inverseFlattening = 0 )
    { 
      this->semimajorAxis = semimajorAxis;
      this->inverseFlattening = inverseFlattening;
    }

    virtual ~CEllipsoid(void) {}
  public:
    //! Calculate the semi-major axis.
    double inline SemimajorAxis ( void ) { return semimajorAxis; }
    //! Calculate the semi minor axis.
    double inline SemiminorAxis ( void ) { return semimajorAxis * ( 1.0 - Flattening() ); }
    //! Calculate the flattening.
    double inline Flattening ( void ) { return 1.0 / inverseFlattening; }
    //! Calculate the inverse flattening.
    double inline InverseFlattening ( void ) { return inverseFlattening; }
    //! Calculate the eccentricity squared.
    double inline EccentricitySq ( void ) 
    { 
        double f = Flattening();
        return (2.0 - f) * f;
    }
    //! Calculate the eccentricity.
    double inline Eccentricity ( void ) 
    { 
        return std::sqrt(EccentricitySq());
    }
    //! Calculate the second eccentricity.
    double inline SecondEccentricity ( void ) 
    { 
        double e2 = EccentricitySq();
        double se = e2 / ( 1.0 - e2);
        return std::sqrt(se);
    }
    //! Calculate N = (a-b)/(a+b)
    double inline N ( void )
    {
      double a = SemimajorAxis();
      double b = SemiminorAxis();
      return ( a - b ) / ( a + b );
    }
    //! Calculate e'^2 = (a^2-b^2)/b^2
    double inline e_dash_sq ( void )
    {
      double a = SemimajorAxis();
      double b = SemiminorAxis();
      return ( a*a - b*b ) / ( b*b );
    }
    //! Calculate the mean length of one degree of latitude.
    double inline G ( void )
    {
      double n = N();
      double n2 = n * n;
      double n3 = n * n2;
      double n4 = n * n3;
      double a = SemimajorAxis();
      static const double dtr = std::atan((double)1.0) / 45.0;
      return a*(1.0-n)*(1.0-n2)*(1.0+(9.0*n2)/4.0+255.0*n4/64.0)*dtr;
    }

  private:
    //! Semimajor axis (m).
    double semimajorAxis;
    //! Inverse flattening.
    double inverseFlattening;
  };

  // Sone standard ellipsoids.

  //! Airy 1830 (semi-major axis, inverse flattening)
  static CEllipsoid AIRY1830(6377563.396, 299.3249646); 
  //! Modified Airy (semi-major axis, inverse flattening)
  static CEllipsoid MODIFIED_AIRY(6377340.189, 299.3249646); 
  //! Australian National (semi-major axis, inverse flattening)
  static CEllipsoid AUSTRALIAN_NATIONAL(6378160, 298.25);  
  //! Bessel 1841 (Namibia) (semi-major axis, inverse flattening)
  static CEllipsoid BESSEL_1841_NAMIBIA(6377483.865, 299.1528128);  
  //! Bessel 1841 (semi-major axis, inverse flattening)
  static CEllipsoid BESSEL_1841(6377397.155, 299.1528128); 
  //! Clarke 1866 (semi-major axis, inverse flattening)
  static CEllipsoid CLARKE_1866(6378206.4, 294.9786982); 
  //! Clarke 1880 (semi-major axis, inverse flattening)
  static CEllipsoid CLARKE_1880(6378249.145, 293.465); 
  //! Everest (India 1830) (semi-major axis, inverse flattening)
  static CEllipsoid EVEREST_INDIA_1830(6377276.345, 300.8017);
  //! Everest (Sabah Sarawak) (semi-major axis, inverse flattening)
  static CEllipsoid EVEREST_SABAH_SARAWAK(6377298.556, 300.801);  
  //! Everest (India 1956) (semi-major axis, inverse flattening)
  static CEllipsoid EVEREST_INDIA_1956(6377301.243, 300.8017); 
  //! Everest (Malaysia 1969) (semi-major axis, inverse flattening)
  static CEllipsoid EVEREST_MALAYSIA_1969(6377295.664, 300.8017);  
  //! Everest (Malay. & Sing) (semi-major axis, inverse flattening)
  static CEllipsoid EVEREST_MALAYSIA_SINGAPORE(6377304.063, 300.8017); 
  //! Everest (Pakistan) (semi-major axis, inverse flattening)
  static CEllipsoid EVEREST_PAKISTAN(6377309.613, 300.8017); 
  //! Modified Fischer 1960 (semi-major axis, inverse flattening)
  static CEllipsoid MODIFIED_FISCHER_1960(6378155.0, 298.3); 
  //! Helmert 1906 (semi-major axis, inverse flattening)
  static CEllipsoid HELMERT_1906(6378200.0, 298.3);
  //! Hough 1960 (semi-major axis, inverse flattening)
  static CEllipsoid HOUGH_1960(6378270.0, 297.0);
  //! Indonesian 1974 (semi-major axis, inverse flattening)
  static CEllipsoid INDONESIAN_1974(6378160.0,298.247);
  //! International 1924 (semi-major axis, inverse flattening)
  static CEllipsoid INTERNATIONAL_1924(6378388.0, 297.0); 
  //! Krassovsky 1940 (semi-major axis, inverse flattening)
  static CEllipsoid KRASSOVSKY_1940(6378245.0, 298.3);
  //! GRS 80 (semi-major axis, inverse flattening)
  static CEllipsoid GRS_80(6378137.0, 298.257222101);
  //! South American 1969 (semi-major axis, inverse flattening)
  static CEllipsoid SOUTH_AMERICAN_1969(6378160, 298.25);  
  //! WGS 72 (semi-major axis, inverse flattening)
  static CEllipsoid WGS72(6378135.0,298.26); 
  //! WGS 84 (semi-major axis, inverse flattening)
  static CEllipsoid WGS84(6378137.0,298.257223563); 


}
#endif // ELLIPSOID_H
