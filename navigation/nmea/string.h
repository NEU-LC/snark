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

/// @author vsevolod vlaskine

#ifndef SNARK_NAVIGATION_NMEA_STRING_H_
#define SNARK_NAVIGATION_NMEA_STRING_H_

#include <string>
#include <comma/base/exception.h>
#include <comma/csv/ascii.h>
#include <comma/string/string.h>

namespace snark { namespace nmea {

class string
{
    public:
        string( const std::string& s );
        
        bool valid() const;
        
        bool complete() const;
        
        const std::string& type() const;
        
        template < typename T > class as;
        
    private:
        template < typename T > friend class as;
        bool valid_;
        bool complete_;
        std::vector< std::string > values_;
};

template < typename T >
class string::as
{
    public:
        T from( const string& m );
        
    private:
        comma::csv::ascii< T > ascii_;
};

template < typename T >
T string::as< T >::from( const string& s )
{
    if( !s.valid() ) { COMMA_THROW( comma::exception, "checksum check failed on: " << comma::join( s.values_, ',' ) ); }
    if( !s.complete() ) { COMMA_THROW( comma::exception, "incomplete message: " << comma::join( s.values_, ',' ) ); }
    if( T::value_type::name() != s.type() ) { COMMA_THROW( comma::exception, "expected message of type: " << T::value_type::name() << ", got: " << s.type() ); }
    return ascii_.get( s.values_ );
}
    
} } // namespace snark { namespace nmea {

#endif // SNARK_NAVIGATION_NMEA_STRING_H_
