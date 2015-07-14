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

#if !defined(MAPGRID_H)
#define MAPGRID_H

#include <cmath>

//! Mapping grids for the earth.
namespace snark { namespace detail { namespace MapGrid {

  //! Stores constants for the map grid.
  /*!
  CLASS
    CMapGrid

    The parameters must be set via the constructor.

    The available map grids are:
    <div align="center">
      <center>
      <table border="1" cellpadding="0" cellspacing="0" style="border-collapse: collapse" bordercolor="#111111" id="AutoNumber1">
        <tr>
          <td width="64">Name</td>
          <td width="78">False<br>
          Easting<br>
          (m)</td>
          <td width="103">False<br>
          Northing<br>
          (m)</td>
          <td width="80">Central<br>
          Scale<br>
          Factor</td>
          <td width="101">Zone<br>
          Width<br>
          (degrees)</td>
          <td width="107">Central<br>
          Meridian <br>
          of Zone 1 <br>
          (degrees)</td>
        </tr>
        <tr>
          <td width="64">MGA</td>
          <td width="78">500000</td>
          <td width="103">10000000</td>
          <td width="80">0.9996</td>
          <td width="101">6</td>
          <td width="107">-177</td>
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
  class CMapGrid
  {
  public:
    //! Construct a map grid.
    /*!
      @param falseEasting - false easting in meters.
      @param falseNorthing - false northing in meters.
      @param centralScaleFactor - central scale factor.
      @param zoneWidth - zone width in degrees.
      @param centralMeridianZone1 - the central meridian of zone 1 in degrees.
    */
    CMapGrid ( 
      double const & falseEasting = 0,
      double const & falseNorthing = 0,
      double const & centralScaleFactor = 0,
      double const & zoneWidth = 0,
      double const & centralMeridianZone1 = 0 )
    { 
      this->falseEasting = falseEasting;
      this->falseNorthing = falseNorthing;
      this->centralScaleFactor = centralScaleFactor;
      this->zonewWidth = zoneWidth;
      this->centralMeridianZone1 = centralMeridianZone1;
    }

    virtual ~CMapGrid(void) {}

  public:
    //! Calculate the false easting in meters.
    double inline FalseEasting ( void ) { return falseEasting; }
    //! Calculate the false northing in meters.
    double inline FalseNorthing ( void ) { return falseNorthing; }
    //! Calculate the central scale factor = K0
    double inline CentralScaleFactor ( void ) { return centralScaleFactor; }
    //! Calculate the zone width in degrees.
    double inline ZoneWidth ( void ) { return zonewWidth; }
    //! Calculate the central meridian of zone 1 in degrees.
    double inline CentralMeridianZone1 ( void ) { return centralMeridianZone1; }
    //! Calculate the longitude of the western edge of zone zero in degrees.
    double inline LWEZone0 ( void ) 
      { return CentralMeridianZone1() - 1.5 * ZoneWidth(); }
    //! Calculate the central meridian of zone zero in degrees.
    double inline CMZone0 ( void ) 
      { return LWEZone0() + ZoneWidth() / 2.0; }

  private:
    //! False easting in meters.
    double falseEasting;
    //! False northing in meters.
    double falseNorthing;
    //! Central scale factor.
    double centralScaleFactor;
    //! Zone width in degrees.
    double zonewWidth;
    //! The central meridian of zone 1 in degrees.
    double centralMeridianZone1;
  };


  //! MGA - Mapping grid of Australia
  /*!  
      Parameters are:
      - falseEasting - false easting in meters.
      - falseNorthing - false northing in meters.
      - centralScaleFactor - central scale factor.
      - zoneWidth - zone width in degrees.
      - centralMeridianZone1 - the central meridian of zone 1 in degrees.
  */
  static CMapGrid MGA(500000.0, 10000000, 0.9996, 6, -177);


} } } // namespace snark { namespace detail { namespace MapGrid {

#endif // MAPGRID_H
