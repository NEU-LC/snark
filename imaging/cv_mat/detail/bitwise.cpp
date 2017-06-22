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

#include "bitwise.h"

#include <boost/regex.hpp>
#include <boost/algorithm/string/erase.hpp>

namespace snark{ namespace cv_mat {

namespace bitwise
{
    std::string tabify_bitwise_ops( const std::string & s )
    {
        static const auto & not_regex = boost::regex( "((\\[|^|\\s+)not\\s+)" );
        static const auto & and_regex = boost::regex( "(\\s+and\\s+)" );
        static const auto & xor_regex = boost::regex( "(\\s+xor\\s+)" );
        static const auto & or_regex  = boost::regex( "(\\s+or\\s+)" );
        return boost::algorithm::erase_all_copy( boost::regex_replace( boost::regex_replace( boost::regex_replace( boost::regex_replace( s, or_regex, "\\tor\\t" ), xor_regex, "\\txor\\t" ), and_regex, "\\tand\\t" ), not_regex, "\\tnot\\t" ), " " );
    };

} // namespace bitwise

} } // namespace snark{ namespace cv_mat {
