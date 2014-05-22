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
// 3. All advertising materials mentioning features or use of this software
//    must display the following acknowledgement:
//    This product includes software developed by the The University of Sydney.
// 4. Neither the name of the The University of Sydney nor the
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

#ifndef SNARK_OCEAN_PACKED_DATA
#define SNARK_OCEAN_PACKED_DATA
#include <comma/base/types.h>
#include <comma/packed/packed.h>
#include <boost/array.hpp>
#include <vector>
#include <iomanip>
#include <iostream>
#include <boost/static_assert.hpp>
#include <boost/type_traits.hpp>

namespace snark { namespace ocean { namespace packed {
    
struct type_info : public comma::packed::packed_struct< type_info, 4 >
{
    comma::packed::string< 1, '$' > start;
    comma::packed::string< 1, '0' > category;
    comma::packed::casted< unsigned int, 1 > controller_id;
    comma::packed::casted< unsigned int, 1 > battery_id;
};
    
struct element : public comma::packed::packed_struct< element, 8 >
{
    comma::packed::string< 1, ',' > comma1;
    comma::packed::ascii_hex< comma::uint16, 2 > address;
    comma::packed::string< 1, ',' > comma2;
    comma::packed::ascii_hex< comma::uint16, 4 > value;
};

template < std::size_t N >
struct packet
{
    ocean::packed::type_info type;
    boost::array< ocean::packed::element, N > values;
    comma::packed::string< 1, '%' > padding;
    comma::packed::ascii_hex< comma::uint16, 2 > crc;
};

} } } // namespace snark { namespace ocean { namespace packed {

#endif //  SNARK_OCEAN_PACKED_DATA
