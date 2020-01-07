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

#include "reduce_partitions.h"

#include <comma/base/exception.h>
#include <comma/base/types.h>
#include <comma/string/split.h>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/functional/hash.hpp>
#include <boost/lexical_cast.hpp>
#include <opencv2/core/mat.hpp>

#include <iostream>
#include <map>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace snark {
namespace cv_mat {
namespace impl {

class graph
{
    public:
        template < typename N >
        graph(cv::Mat m, unsigned int channel, int background): min_id_( background + 1 ) {
            N* ptr;
            N* ptr_below;
            for (auto i = 0; i < m.rows - 1; ++i) {
                ptr = m.ptr<N>(i);
                ptr_below = m.ptr<N>(i + 1);
                for (auto j = channel; j < ( m.cols - 1 ) * m.channels() ; j += m.channels()) {
                    const auto& val = ptr[j];
                    if (val == background) continue;
                    auto& id = ids_.emplace( std::make_pair, val, val ).first.second;
                    adjacency_.emplace(std::make_pair( &id, std::make_pair(std::set<N*>(), min_id_)));
                    const auto& val_right = ptr[j + m.channels()];
                    const auto& val_below = ptr_below[j];
                    if (val != val_below) {
                        insert_helper_<N>(val_below);
                    }
                    if (val != val_right) {
                        insert_helper_< N >(val_right);
                    }
                }
            }
        }        

    private:
        std::unordered_map< comma::int32*, std::pair< std::unordered_set< comma::int32* >, comma::int32 > > adjacency_;
        std::unordered_map< comma::int32, comma::int32 > ids_;
        int min_id_;

        template < typename N >
        void insert_helper_(N val) {
            auto it = std::find_if(
                std::begin(adjacency_), std::end(adjacency_),
                [val](std::pair<const std::unique_ptr<N>, std::pair<std::set<N*>, N>>& p) { return *(p.first) == val; });
            auto raw_ptr = it->first.get();
            adjacency_.emplace(std::make_pair(std::unique_ptr<N>(new N(val)), std::make_pair(std::set<N*>{raw_ptr}, 0)));
        };
};


template <typename N>
struct graph_old {
    graph_old(cv::Mat m, unsigned int channel) {
        const auto& channels = m.channels();

        auto nRows = m.rows - 1;
        auto nCols = (m.cols - 1) * channels;

        N* ptr;
        N* ptr_below;
        for (auto i = 0; i < nRows; ++i) {
            ptr = m.ptr<N>(i);
            ptr_below = m.ptr<N>(i + 1);
            for (auto j = channel; j < nCols; j += channels) {
                const auto& val = ptr[j];
                if (val == BACKGROUND) continue;
                adj_list_.emplace(std::make_pair(std::unique_ptr<N>(new N(val)), std::make_pair(std::set<N*>(), 0)));
                const auto& val_right = ptr[j + channels];
                const auto& val_below = ptr_below[j];
                if (val != val_below) {
                    insert_helper(val_below);
                }
                if (val != val_right) {
                    insert_helper(val_right);
                }
            }
        }
    }
    void insert_helper(N val) {
        auto it = std::find_if(
            std::begin(adj_list_), std::end(adj_list_),
            [val](std::pair<const std::unique_ptr<N>, std::pair<std::set<N*>, N>>& p) { return *(p.first) == val; });
        auto raw_ptr = it->first.get();
        adj_list_.emplace(std::make_pair(std::unique_ptr<N>(new N(val)), std::make_pair(std::set<N*>{raw_ptr}, 0)));
    };

    std::map<std::unique_ptr<N>, std::pair<std::set<N*>, N>> adj_list_;
};

template <typename H>
std::pair<H, cv::Mat> reduce_partitions<H>::operator()(std::pair<H, cv::Mat> m) {
    switch (m.second.depth()) {
        case CV_8U: {
            graph<unsigned char> g(m.second, channel_);
            std::cout << "Hello\n";
            break;
        }
        case CV_8S: {
            graph<char> g(m.second, channel_);
            break;
        }
        case CV_16U: {
            graph<comma::uint16> g(m.second, channel_);
            break;
        }
        case CV_16S: {
            graph<comma::int16> g(m.second, channel_);
            break;
        }
        case CV_32S: {
            graph<comma::int32> g(m.second, channel_);
            break;
        }
        default: { COMMA_THROW(comma::exception, "expected image data, got unsupported value: " << m.second.type()); }
    }
    return {};
}

template <typename H>
std::pair<typename reduce_partitions<H>::functor_t, bool> reduce_partitions<H>::make(const std::string& options) {
    unsigned int channel = 0;
    if (!options.empty()) {
        const auto& tokens = comma::split(options, ',');
        if (tokens.size() > 1) {
            COMMA_THROW(comma::exception, "reduce-partitions: expected options, got: '" << options << "'");
        }
        try {
            channel = boost::lexical_cast<decltype(channel)>(tokens[0]);
        } catch (std::exception& e) {
            COMMA_THROW(comma::exception,
                        "reduce-partitions: expected <channel>, got: '" << options << "'; " << e.what());
        }
        if (channel > 2) {
            COMMA_THROW(comma::exception,
                        "reduce-partitions: expected channel 0, 1, 2, got unsupported value: " << channel);
        }
    }
    return std::make_pair(reduce_partitions<H>(channel), true);
}

template <typename H>
typename std::string reduce_partitions<H>::usage(unsigned int indent) {
    std::string offset(indent, ' ');
    std::ostringstream oss;
    oss << offset << "reduce-partitions=[<channel>]; partition channel number in image; default: 0\n";
    return oss.str();
}

template class reduce_partitions<boost::posix_time::ptime>;
template class reduce_partitions<std::vector<char>>;

}  // namespace impl
}  // namespace cv_mat
}  // namespace snark
