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
#include "contraharmonic.h"
#include "utils.h"

#include <comma/base/exception.h>
#include <comma/string/split.h>
#include <tbb/parallel_for.h>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/lexical_cast.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace snark {
namespace cv_mat {
namespace impl {

template <typename T>
struct Handle {
    Handle(cv::Mat in, cv::Mat &out, double power, int kernel) : in_{in}, out_{out}, power_{power}, kernel_{kernel} {};

    void operator()(int start, int end) {
        end = std::min(end, in_.rows);
        std::vector<double> numerator(in_.channels(), 0);
        std::vector<double> denominator(in_.channels(), 0);
        for (; start < end; ++start) {
            for (int col = 0; col < in_.cols; ++col) {
                for (int kernel_y = -(kernel_ / 2); kernel_y < std::ceil(static_cast<double>(kernel_) / 2.);
                     ++kernel_y) {
                    int y = start + kernel_y;
                    if (y >= 0 && y < in_.rows) {
                        const T *row_ptr = in_.template ptr<T>(y);
                        for (int kernel_x = -(kernel_ / 2); kernel_x < std::ceil(static_cast<double>(kernel_) / 2.);
                             ++kernel_x) {
                            int x = col + kernel_x;
                            if (x >= 0 && x < in_.cols) {
                                for (int channel = 0; channel < in_.channels(); ++channel) {
                                    numerator[channel] += std::pow(row_ptr[in_.channels() * x + channel], power_ + 1);
                                    denominator[channel] += std::pow(row_ptr[in_.channels() * x + channel], power_);
                                }
                            }
                        }
                    }
                }
                T *row_ptr = out_.template ptr<T>(start);
                for (int channel = 0; channel < in_.channels(); ++channel) {
                    row_ptr[out_.channels() * col + channel] = numerator[channel] / denominator[channel];
                    if (std::isnan(row_ptr[out_.channels() * col + channel])) {
                        row_ptr[out_.channels() * col + channel] = 0;
                    }
                    numerator[channel] = 0;
                    denominator[channel] = 0;
                }
            }
        }
    };

    cv::Mat in_;
    cv::Mat out_;
    double power_;
    int kernel_;
};

template <typename H>
contraharmonic<H>::contraharmonic(int kernel, double power) : kernel_{kernel}, power_{power} {}

template <typename H>
std::pair<H, cv::Mat> contraharmonic<H>::operator()(std::pair<H, cv::Mat> m) {
    cv::Mat in{m.second};
    if (!m.second.isContinuous()) {
        COMMA_THROW(comma::exception, "matrix not continuous; non-continous image data not supported")
    }

    int sizes[2]{m.second.rows, m.second.cols};
    cv::Mat out(m.second.dims, sizes, m.second.type());

    size_t n = std::thread::hardware_concurrency() / 2;
    int chunk_size = std::ceil(static_cast<double>(m.second.rows) / static_cast<double>(n));

    switch (m.second.depth()) {
        case CV_8U: {
            auto handle_row = Handle<uint8_t>(m.second, out, power_, kernel_);
            tbb::parallel_for(size_t{0}, n, [&chunk_size, &handle_row](size_t i) {
                handle_row(chunk_size * i, chunk_size * (i + 1));
            });
            break;
        }
        case CV_8S: {
            auto handle_row = Handle<int8_t>(m.second, out, power_, kernel_);
            tbb::parallel_for(size_t{0}, n, [&chunk_size, &handle_row](size_t i) {
                handle_row(chunk_size * i, chunk_size * (i + 1));
            });
            break;
        }
        case CV_16U: {
            auto handle_row = Handle<uint16_t>(m.second, out, power_, kernel_);
            tbb::parallel_for(size_t{0}, n, [&chunk_size, &handle_row](size_t i) {
                handle_row(chunk_size * i, chunk_size * (i + 1));
            });
            break;
        }
        case CV_16S: {
            auto handle_row = Handle<int16_t>(m.second, out, power_, kernel_);
            tbb::parallel_for(size_t{0}, n, [&chunk_size, &handle_row](size_t i) {
                handle_row(chunk_size * i, chunk_size * (i + 1));
            });
            break;
        }
        case CV_32S: {
            auto handle_row = Handle<int32_t>(m.second, out, power_, kernel_);
            tbb::parallel_for(size_t{0}, n, [&chunk_size, &handle_row](size_t i) {
                handle_row(chunk_size * i, chunk_size * (i + 1));
            });
            break;
        }
        case CV_32F: {
            auto handle_row = Handle<float>(m.second, out, power_, kernel_);
            tbb::parallel_for(size_t{0}, n, [&chunk_size, &handle_row](size_t i) {
                handle_row(chunk_size * i, chunk_size * (i + 1));
            });
            break;
        }
        case CV_64F: {
            auto handle_row = Handle<double>(m.second, out, power_, kernel_);
            tbb::parallel_for(size_t{0}, n, [&chunk_size, &handle_row](size_t i) {
                handle_row(chunk_size * i, chunk_size * (i + 1));
            });
            break;
        }
        default: {
            COMMA_THROW(comma::exception, "unknown image data type");
            break;
        }
    }

    return std::make_pair(m.first, out);
}

template <typename H>
std::pair<typename contraharmonic<H>::functor_t, bool> contraharmonic<H>::make(const std::string &options) {
    // options -> square,7,1 whatever contraharmonic=<options>
    double power{};
    std::string shape{"square"};
    int kernel{};
    if (!options.empty()) {
        const auto &s = comma::split(options, ',');
        if (s.size() != 3) {
            COMMA_THROW(comma::exception, "contraharmonic: expected shape,kernel,power got: " << options);
        }
        shape = s[1];
        if (shape != "square") {
            COMMA_THROW(comma::exception, "contraharmonic: expected shape=square got: " << shape);
        }
        power = boost::lexical_cast<decltype(power)>(s[0]);
        kernel = boost::lexical_cast<decltype(kernel)>(s[2]);
    }
    return std::make_pair(contraharmonic<H>{kernel, power}, true);  // todo
}

template <typename H>
typename std::string contraharmonic<H>::usage(unsigned int indent) {
    std::string offset(indent, ' ');
    std::ostringstream oss;
    oss << offset << "contraharmonic=<power>,<kernel_shape>,<kernel_geometry>, e.g: contraharmonic=3,square,5\n";
    oss << offset
        << "               <power>: order of the filter; positive values remove pepper noise; negative values remove "
           "salt noise\n";
    oss << offset
        << "               <kernel_shape>: shape of kernel filter; default: square; todo: support other shapes, e.g. "
           "rectangle\n";
    oss << offset << "               <kernel_geometry>:\n";
    oss << offset << "                   square,<side>, e.g: 'square,5'\n";
    return oss.str();
}

template class contraharmonic<boost::posix_time::ptime>;
template class contraharmonic<std::vector<char>>;

}  // namespace impl
}  // namespace cv_mat
}  // namespace snark
