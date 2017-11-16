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
//    documentation and/or other materials provided with the distribution.ll
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


#include <boost/array.hpp>
#include "db.h"
#include "../laser_map.h"

namespace snark {  namespace velodyne { namespace hdl64 {

// todo
// - move hdl64-related stuff from velodyne to velodyne/hdl64, if easy
// - implement velodyne::puck::laser_map
// - velodyne-to-mesh --puck: plug in puck::laser_map, test on visualized puck data
// - velodyne-to-mesh-ground --puck: plug in puck::laser_map, test on visualized puck data
// - points-match-and-align --puck: pass --puck to relevant utilities, test on puck data
    
/// orders lasers by elevation
class laser_map : public velodyne::laser_map
{
    public:
        /// constructor
        laser_map( const snark::velodyne::hdl64::db& db );

        /// take laser id, return index by elevation
        unsigned int id_to_index( unsigned int i ) const;

        /// take index by elevation, return laser id
        unsigned int index_to_id( unsigned int i ) const;
        
        unsigned int size() const { return 64; }
        
    private:
        boost::array< unsigned int, 64 > indices_;
        boost::array< unsigned int, 64 > ids_;
};
    
} } } // namespace snark {  namespace velodyne { namespace hdl64 {
    
