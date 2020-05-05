// This file is provided in addition to snark and is not an integral
// part of snark library.
// Copyright (c) 2019 Steven Potiris, Vsevolod Vlaskine
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
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

/// @author steven potiris

#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/range/adaptor/indexed.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <comma/base/exception.h>
#include <comma/base/types.h>
#include <comma/string/split.h>
#include "pad.h"

namespace snark { namespace cv_mat { namespace filters { namespace pad {

static std::map< std::string, int > border_fill_types = boost::assign::map_list_of 
    ("constant", cv::BORDER_CONSTANT)
    ("replicate", cv::BORDER_REPLICATE)
    ("wrap", cv::BORDER_WRAP)
    ("reflect", cv::BORDER_REFLECT_101);

template < typename H >
pad< H >::pad(int top, int bottom, int left, int right, int border_type, const cv::Scalar& fill)
        : top_(top), bottom_(bottom), left_(left), right_(right), border_type_(border_type), fill_(fill) {
}

template < typename H >
std::pair< typename pad< H >::functor_t, bool > pad< H >::make(const std::string &options) {
    std::vector< std::string > s = comma::split(options, ',');
    if (s.size() < 4) { COMMA_THROW(comma::exception, "pad: expected at least four values; got: '" << options << "'" ); }
    if (s.size() > 9) { COMMA_THROW(comma::exception, "pad: expected at most nine elements; got: '" << options << "'" ); }
    int top = boost::lexical_cast< int >(s[0]);
    int bottom = boost::lexical_cast< int >(s[1]);
    int left = boost::lexical_cast< int >(s[2]);
    int right = boost::lexical_cast< int >(s[3]);
    std::string border_type_str = (s.size() >= 5) ? s[4] : "constant";
    if (!border_fill_types.count(border_type_str)) { COMMA_THROW(comma::exception, "pad: expected border type; got: '" << border_type_str << "'"); }
    cv::Scalar fill(0, 0, 0, 0);
    int i = 0;
    for (auto iter = s.begin() + 5; iter < s.end(); iter += 1, i++) { fill[i] = boost::lexical_cast<double>(*iter); }
    return std::make_pair(pad< H >(top, bottom, left, right, border_fill_types[border_type_str], fill), true);
}

template < typename H >
typename std::string pad< H >::usage(unsigned int indent) {
    std::string offset(indent, ' ');
    std::ostringstream oss;
    oss << offset << "pad=<top>,<bottom>,<left>,<right>[,<border_type>][,<fill>]; see cv::copyMakeBorder for details" << std::endl;
    oss << offset << "    <top>,<bottom>,<left>,<right>: number of pixels to pad on each side" << std::endl;
    oss << offset << "    <border_type>: one of: 'constant', 'replicate', 'wrap', 'reflect'" << std::endl;
    oss << offset << "    <fill>: vector of ints (colour) to fill the padding with" << std::endl;
    return oss.str();
}

template < typename H >
std::pair< H, cv::Mat > pad< H >::operator()(std::pair< H, cv::Mat > m) {
    std::pair< H, cv::Mat > n;
    n.first = m.first;
    cv::copyMakeBorder(m.second, n.second, top_, bottom_, left_, right_, border_type_, fill_);
    return n;
}

template class pad< boost::posix_time::ptime >;
template class pad< std::vector< char > >;

} } } } // namespace snark { namespace cv_mat { namespace impl { namespace pad {
