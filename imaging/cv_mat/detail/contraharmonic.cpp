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
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/lexical_cast.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <tbb/parallel_for.h>

#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace snark {
namespace cv_mat {
namespace impl {

template <typename H>
contraharmonic<H>::contraharmonic(int kernel, double power)
    : kernel_{kernel}, power_{power} {}  // todo

template <typename H>
std::pair<H, cv::Mat> contraharmonic<H>::operator()(std::pair<H, cv::Mat> m) {
  int sizes[2]{m.second.rows, m.second.cols};
  cv::Mat im_out(m.second.dims, sizes, m.second.type());
  if (m.second.depth() != CV_8U) {
    COMMA_THROW(comma::exception,
                "todo: support for depth: " << m.second.depth());
  }
  // for (int row = kernel_ / 2; row < m.second.rows - kernel_ / 2 - 1; ++row) {
  //   for (int col = kernel_ / 2; col < m.second.cols - kernel_ / 2 - 1; ++col) {
  //     double num{0};
  //     double den{0};
  //     for (int channel = 0; channel < m.second.channels(); ++channel) {
  //       for (int y = -(kernel_ / 2); y <= (kernel_ / 2); ++y) {
  //         std::cout << y <<'\n';
  //         for (int x = -(kernel_ / 2); x <= (kernel_ / 2); ++x) {
  //           num += std::pow(
  //               m.second.template at<cv::Vec3b>(row + y, col + x)[channel],
  //               power_ + 1);
  //           den += std::pow(
  //               m.second.template at<cv::Vec3b>(row + y, col + x)[channel],
  //               power_);
  //         }
  //       }
  //       im_out.template at<cv::Vec3b>(row, col)[channel] = num / den;
  //     }
  //   }
  // }

  std::vector<double> numerator(m.second.channels(), 0);
  std::vector<double> denominator(m.second.channels(), 0);
  for (int row = 0; row < m.second.rows; ++row) {
    for (int col = 0; col < m.second.cols; ++col) {
      for (int kernel_y = -(kernel_ / 2); kernel_y < std::ceil(static_cast<double>(kernel_) / 2.); ++kernel_y) {
        for (int kernel_x = -(kernel_ / 2); kernel_x < std::ceil(static_cast<double>(kernel_) / 2.); ++kernel_x) {
          int y = row + kernel_y;
          int x = col + kernel_x;
          if (y >= 0 && y < m.second.rows && (x >= 0 && x < m.second.cols)) {
            const auto& vec = m.second.template at <cv::Vec3b>(y, x);
            for (int channel = 0; channel < m.second.channels(); ++channel) {
              numerator[channel] += std::pow(vec[channel], power_ + 1);
              denominator[channel] += std::pow(vec[channel], power_);
            }
          }
        }
      }
      auto& vec_out = im_out.template at<cv::Vec3b>(row, col);
      for (int channel = 0; channel < m.second.channels(); ++channel) {
        vec_out[channel] = numerator[channel] / denominator[channel];
        numerator[channel] = 0;
        denominator[channel] = 0;
      }
    }
  }

  // tbb::parallel_for(tbb::blocked_range<std::size_t>(0, std::thread::hardware_concurrency()), [&](const tbb::blocked_range<std::size_t>& r) {
  //   for (auto i = r.begin(); i != r.end(); ++i) {
  //     std::this_thread::sleep_for(std::chrono::seconds(5));
  //   }
  // });

  return std::make_pair(m.first, im_out);
}

template <typename H>
std::pair<typename contraharmonic<H>::functor_t, bool> contraharmonic<H>::make(
    const std::string &options) {
  // options -> square,7,1 whatever contraharmonic=<options>
  std::string shape{"square"};
  int kernel{};
  double power{};
  if (!options.empty()) {
    const auto &s = comma::split(options, ',');
    if (s.size() != 3) {
      COMMA_THROW(
          comma::exception,
          "contraharmonic: expected shape,kernel,power got: " << options);
    }
    shape = s[0];
    if (shape != "square") {
      COMMA_THROW(comma::exception,
                  "contraharmonic: expected shape=rectangle got: " << shape);
    }
    kernel = boost::lexical_cast<decltype(kernel)>(s[1]);
    power = boost::lexical_cast<decltype(power)>(s[2]);
  }
  return std::make_pair(contraharmonic<H>{kernel, power}, true);  // todo
}

template <typename H>
typename std::string contraharmonic<H>::usage(unsigned int indent) {
  std::string offset(indent, ' ');
  std::ostringstream oss;
  oss << offset << "contraharmonic[=<options>]\n";
  oss << offset << "    <options>\n";
  oss << offset
      << "        shape:<shape>: shape of kernel filter; default: square\n";
  oss << offset
      << "        kernel:<value>: size of kernel to apply filter to image\n";
  oss << offset
      << "        power:<value>: order of the filter; positive values remove "
         "pepper noise; negative values remove salt noise\n";
  return oss.str();
}

template class contraharmonic<boost::posix_time::ptime>;
template class contraharmonic<std::vector<char>>;

}  // namespace impl
}  // namespace cv_mat
}  // namespace snark
