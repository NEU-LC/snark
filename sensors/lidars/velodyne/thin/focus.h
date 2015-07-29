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


#ifndef SNARK_SENSORS_VELODYNE_THIN_FOCUS
#define SNARK_SENSORS_VELODYNE_THIN_FOCUS

#include <map>
#include <boost/shared_ptr.hpp>
#include "region.h"

namespace snark {  namespace velodyne { namespace thin {

/// focus on particular region in the velodyne scan
class focus
{
    public:
        focus( double rate = 1.0, double ratio = 1.0 );
        template < typename Random >
        bool has( double range, double bearing, double elevation, Random& random ) const;
        double rate_in_focus() const;
        double rate_out_of_focus() const;
        double coverage() const;
        void insert( std::size_t id, region* r );
        void erase( std::size_t id );

    private:
        typedef std::map< std::size_t, boost::shared_ptr< region > > Map;
        double m_rate;
        double m_ratio;
        Map m_regions;
        double m_rate_in_focus;
        double m_rate_out_of_focus;
        void update();
};

template < typename Random >
bool focus::has( double range, double bearing, double elevation, Random& random ) const
{
    double r = random();
    for( typename Map::const_iterator it = m_regions.begin(); it != m_regions.end(); ++it )
    {
        if( it->second->has( range, bearing, elevation ) ) { return r < m_rate_in_focus; }
    }
    return r < m_rate_out_of_focus;
}

} } } // namespace snark {  namespace velodyne { namespace thin {

#endif // #ifndev SNARK_SENSORS_VELODYNE_THIN_FOCUS

