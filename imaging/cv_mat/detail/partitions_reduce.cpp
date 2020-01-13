// This file is provided in addition to snark and is not an integral
// part of snark library.
// Copyright (c) 2019 Vsevolod Vlaskine
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

// snark is a generic and flexible library for robotics research
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

#include "partitions_reduce.h"

#include <comma/base/exception.h>
#include <comma/base/types.h>
#include <comma/string/split.h>
#include <tbb/parallel_for.h>
#include <algorithm>
#include <array>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/functional/hash.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace snark {
namespace cv_mat {
namespace impl {

struct vertex {
    bool operator<(const vertex& other) const { return *id < *other.id; };

    comma::int32* id;
    unsigned int degree;
};

struct compare_by_degree {
    bool operator()(const vertex& a, const vertex& b) const { return a.degree > b.degree; }
};

template <typename N>
class graph {
   public:
    graph(cv::Mat m, unsigned int channel, comma::int32 background) : min_id_(background + 1) {
        auto insert_edge = [this](N val, set_t& first_neighbours, comma::int32* first_id_ptr) {
            comma::int32* second_id_ptr = ids_.find(val) == ids_.end() ? &(ids_[val] = val) : &ids_[val];
            set_t* second_neighbours = &adjacency_[second_id_ptr];
            first_neighbours.insert(second_id_ptr);
            second_neighbours->insert(first_id_ptr);
        };

        for (auto i = 0; i < m.rows - 1; ++i) {
            N* ptr = m.ptr<N>(i);
            N* ptr_below = m.ptr<N>(i + 1);
            for (int j = channel; j < (m.cols - 1) * m.channels(); j += m.channels()) {
                N val = ptr[j];
                if (val == background) { continue; }
                N val_below = ptr_below[j];
                N val_right = ptr[j + m.channels()];
                comma::int32* first_id_ptr = ids_.find(val) == ids_.end() ? &(ids_[val] = val) : &ids_[val];
                set_t* first_neighbours = &adjacency_[first_id_ptr];
                if (val != val_below && val_below != background) { insert_edge(val_below, *first_neighbours, first_id_ptr); }
                if (val != val_right && val_right != background) { insert_edge(val_right, *first_neighbours, first_id_ptr); }
            }
        }
    }

    void reduce() {
        std::multimap<vertex, set_t, compare_by_degree> adjacency{};
        for (auto it = adjacency_.begin(); it != adjacency_.end();) {
            auto neighbours = std::move(it->second);
            auto ptr = std::move(it->first);
            *ptr = min_id_;
            it = adjacency_.erase(it);
            if (neighbours.empty()) { continue; }
            *ptr += 6;
            adjacency.emplace(vertex{std::move(ptr), static_cast<unsigned int>(neighbours.size())}, std::move(neighbours));
        }
        for (auto pair : adjacency) {
            std::array<bool, 6> colours_taken{false, false, false, false, false, false};
            for (comma::int32* o_vertex : pair.second) {
                auto colour = *o_vertex - min_id_;  // offset by min id in partitions
                if (colour >= 0 && colour < int(colours_taken.size())) { colours_taken[colour] = true; }
            }
            unsigned int i = 0;
            for( ; i < colours_taken.size() && colours_taken[i]; ++i );
            if( i == colours_taken.size() ) { COMMA_THROW(comma::exception, "partitions-reduce: adjacent vertices have used all 6 available colours"); }
            *pair.first.id = i + min_id_;
        }
    }

    const std::unordered_map<comma::int32, comma::int32>& ids() const { return ids_; }

   private:
    typedef std::unordered_set<comma::int32*> set_t;
    std::unordered_map<comma::int32*, set_t> adjacency_;
    std::unordered_map<comma::int32, comma::int32> ids_;  // old -> new
    comma::int32 min_id_;
};

template <typename H>
template <typename T, int I>
cv::Mat partitions_reduce<H>::process_(cv::Mat m, int type ) {
    graph<T> g(m, channel_, background_);
    g.reduce();
    auto lookup_table = g.ids();
    auto channel = channel_;
    cv::Mat partitions_reduced_channel(m.rows, m.cols, I);
    tbb::parallel_for(size_t(0), size_t(m.rows), [&m, &partitions_reduced_channel, &lookup_table, channel](size_t i) {
        auto ptr_a = m.template ptr<T>(i);
        auto ptr_b = partitions_reduced_channel.ptr<T>(i);
        for (auto j = 0; j < m.cols; ++j) { ptr_b[j] = lookup_table[ptr_a[j * m.channels() + channel]]; }
    });
    if ( !merge_ ) { return partitions_reduced_channel; }
    cv::Mat out( m.rows, m.cols, type );
    std::vector<int> from_to(out.channels() * 2);
    for (auto i = 0; i < out.channels(); ++i) { from_to[i * 2] = from_to[i * 2 + 1] = i; }
    cv::mixChannels(std::vector<cv::Mat>{m, partitions_reduced_channel}, std::vector<cv::Mat>{out}, from_to);
    return out;
}

template <typename H>
std::pair<H, cv::Mat> partitions_reduce<H>::operator()(std::pair<H, cv::Mat> m) {
    if (m.second.channels() > 3) { COMMA_THROW(comma::exception, "partitions-reduce: not more than 3 channels, got " << m.second.channels()); }
    std::pair<H, cv::Mat> out;
    out.first = m.first;
    switch (m.second.depth()) {
        case CV_8U:
            out.second = process_<unsigned char, CV_8UC1>(m.second, CV_8UC(m.second.channels() + 1));
            break;
        case CV_8S:
            out.second = process_<char, CV_8SC1>(m.second, CV_8SC(m.second.channels() + 1));
            break;
        case CV_16U:
            out.second = process_<comma::uint16, CV_16UC1>(m.second, CV_16UC(m.second.channels() + 1));
            break;
        case CV_16S:
            out.second = process_<comma::int16, CV_16SC1>(m.second, CV_16SC(m.second.channels() + 1));
            break;
        case CV_32S:
            out.second = process_<comma::int32, CV_32SC1>(m.second, CV_32SC(m.second.channels() + 1));
            break;
        default:
            COMMA_THROW(comma::exception, "partitions-reduce: expected image depth, got value: " << m.second.depth() << "; not supported (yet?)");
    }
    return out;
}

template <typename H>
std::pair<typename partitions_reduce<H>::functor_t, bool> partitions_reduce<H>::make(const std::string& options) {
    unsigned int channel = 0;
    comma::int32 background = -1;
    bool merge = false;
    if (!options.empty()) {
        const std::vector<std::string>& tokens = comma::split(options, ',');
        try
        {
            switch (tokens.size()) {
                case 0:
                    break;
                case 1:
                    channel = boost::lexical_cast<unsigned int>(tokens[0]);
                    break;
                case 2:
                    if( !tokens[0].empty() ) { channel = boost::lexical_cast<unsigned int>(tokens[0]); }
                    background = boost::lexical_cast<comma::int32>(tokens[1]);
                    break;
                case 3:
                    if ( !tokens[0].empty() ) { channel = boost::lexical_cast<unsigned int>(tokens[0]); }
                    if ( !tokens[1].empty() ) { background = boost::lexical_cast<comma::int32>(tokens[1]); }
                    merge = tokens[2] == "merge";
                    break;
                default:
                    throw;
            }
        }
        catch( std::exception& ex ) { COMMA_THROW( comma::exception, "partitions-reduce: invalid options:: '" << options << "'; " << ex.what()); }
    }
    return std::make_pair(partitions_reduce<H>(channel, background, merge), true);
}

// todo
// - partitions_reduce.h/cpp -> partitions.h/cpp
// - partitions_reduce -> partitions::reduce
// - --help: explain operation

template <typename H>
typename std::string partitions_reduce<H>::usage(unsigned int indent) {
    std::string offset(indent, ' ');
    std::ostringstream oss;
    oss << offset << "partitions-reduce=[<channel>],[<background>],[<merge>]; todo: explain operation\n";
    oss << offset << "    <channel>; partition channel number in image; default: 0\n";
    oss << offset << "    <background>; pixel value that is not assigned any partition; default: -1\n";
    oss << offset << "    <merge>: if present merge reduced partitions channel to original image\n";
    return oss.str();
}

template class partitions_reduce<boost::posix_time::ptime>;
template class partitions_reduce<std::vector<char>>;

}  // namespace impl
}  // namespace cv_mat
}  // namespace snark
