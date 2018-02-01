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


#ifndef SNARK_SENSORS_VELODYNE_IMPL_STREAMTRAITS_H_
#define SNARK_SENSORS_VELODYNE_IMPL_STREAMTRAITS_H_

#include <boost/optional.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "../scan_tick.h"
#include "pcap_reader.h"
#include "proprietary_reader.h"
#include "thin_reader.h"

namespace snark {  namespace velodyne { namespace impl {

template < typename S > struct stream_traits
{
    static const char* read( S& s, std::size_t size ) { return s.read(); } // static const char* read( S& s, std::size_t size ) { return s.read( size ); }

    static boost::posix_time::ptime timestamp( const S& s ) { return s.timestamp(); }

    static void close( S& s ) { s.close(); }

    template < typename P > static std::pair<bool,bool> is_new_scan( scan_tick& tick, const S& s, const P& p ) { return tick.is_new_scan( p, timestamp(s) ); }
};

template <> struct stream_traits< proprietary_reader >
{
    static const char* read( proprietary_reader& s, std::size_t ) { return s.read(); }

    static boost::posix_time::ptime timestamp( const proprietary_reader& s ) { return s.timestamp(); }

    static void close( proprietary_reader& s ) { s.close(); }

    template < typename P > static std::pair<bool,bool> is_new_scan( scan_tick& tick, const proprietary_reader& s, const P& p ) { return tick.is_new_scan( p, timestamp(s) ); }
};

template <> struct stream_traits< pcap_reader >
{
    static const char* read( pcap_reader& s, std::size_t size )
    {
        const char* r = s.read();
        if( r == NULL ) { return NULL; }
        return r + 42; // skip UDP header
    }

    static boost::posix_time::ptime timestamp( const pcap_reader& s ) { return s.timestamp(); }

    static void close( pcap_reader& s ) { s.close(); }

    template < typename P > static std::pair<bool,bool> is_new_scan( scan_tick& tick, const pcap_reader& s, const P& p ) { return tick.is_new_scan( p, timestamp(s) ); }
};

template <> struct stream_traits< thin_reader >
{
    static const char* read( thin_reader& s, std::size_t size ) { return s.read(); } // static const char* read( S& s, std::size_t size ) { return s.read( size ); }

    static boost::posix_time::ptime timestamp( const thin_reader& s ) { return s.timestamp(); }

    static void close( thin_reader& s ) { s.close(); }

    template < typename P > static std::pair<bool,bool> is_new_scan( const scan_tick&, thin_reader& r, const P& ) { return std::pair<bool,bool>(r.is_new_scan(),true); }
};

} } } // namespace snark {  namespace velodyne { namespace impl {

#endif // SNARK_SENSORS_VELODYNE_IMPL_STREAMTRAITS_H_
