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


#pragma once

#include <iostream>
#include <string>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <opencv2/core/core.hpp>
#include <comma/base/types.h>
#include <comma/csv/binary.h>
#include <comma/csv/traits.h>

namespace snark { namespace cv_mat {

class serialization
{
    public:
        /// header for cv::Mat serialization
        struct header
        {
            boost::posix_time::ptime timestamp;
            comma::uint32 rows;
            comma::uint32 cols;
            comma::uint32 type; /// cv::Mat type (see http://opencv.willowgarage.com/documentation/cpp/basic_structures.html)
            comma::uint32 size; /// data size, convenience member equal to rows*cols
            
            header();
            header( const cv::Mat& m );
            header( const std::pair< boost::posix_time::ptime, cv::Mat >& p ); /// constructor
            header( const boost::posix_time::ptime& t, const cv::Mat & p ); /// constructor
            
            typedef std::vector< char > buffer_t;
        };
        
        typedef boost::shared_ptr< comma::csv::binary< header > > binary_type;
        
        /// options, a helper class
        struct options
        {
            std::string fields;
            comma::csv::format format;
            
            comma::uint32 rows;
            comma::uint32 cols;
            std::string type;
            bool no_header;
            bool header_only;
            
            options() : no_header( false ), header_only( false ) {}
            header get_header() const; /// make header (to be used as default)
            static std::string usage();
            static std::string type_usage();
        };
        

        /// default constructor
        serialization();

        /// constructor
        serialization( const std::string& fields, const comma::csv::format& format, bool headerOnly = false, const header& default_header = header() );
        
        /// constructor
        serialization( const options& options );

        /// serialize cv::Mat, return image size in buffer
        std::size_t put( const std::pair< boost::posix_time::ptime, cv::Mat >& m, char* buf ) const;

        /// deserialize cv::Mat, return image size in buffer
        /// @note get timestamp from header()
        std::size_t get( std::pair< boost::posix_time::ptime, cv::Mat >& m, const char* buf ) const;

        /// deserialize cv::Mat, convenience class
        std::pair< boost::posix_time::ptime, cv::Mat > get( const char* buf ) const;
        
        /// deserialize header only
        header get_header( const char* buf ) const;
        
        /// return usage
        static const std::string& usage();

        /// return necessary buffer size
        std::size_t size( const cv::Mat& m ) const;

        /// same as above
        std::size_t size( const std::pair< boost::posix_time::ptime, cv::Mat >& m ) const;
        std::size_t size( const std::pair< header::buffer_t, cv::Mat >& m ) const;

        /// read from stream, if eof, return empty cv::Mat
        std::pair< boost::posix_time::ptime, cv::Mat > read( std::istream& is );
        std::pair< header::buffer_t, cv::Mat > read_with_header( std::istream& is );
        
        /// return last header buffer after read()
        const char* header_buffer() const; // todo
        const std::vector< char >& header_vector() const; // todo

        /// write to stream
        void write( std::ostream& os, const std::pair< boost::posix_time::ptime, cv::Mat >& m, bool flush = true );

        /// c-style write to stdout, to be used if issues seen with write() - see cpp file for details
        void write_to_stdout( const std::pair< boost::posix_time::ptime, cv::Mat >& m, bool flush = true );
        
        /// write to stream
        void write( std::ostream& os, const std::pair< header::buffer_t, cv::Mat >& m, bool flush = true );
        
        /// c-style write to stdout, to be used if issues seen with write() - see cpp file for details
        void write_to_stdout( const std::pair< header::buffer_t, cv::Mat >& m, bool flush = true );
        
        const binary_type& input_binary() const { return m_binary; }

    private:
        boost::shared_ptr< comma::csv::binary< header > > m_binary;
        boost::shared_ptr< comma::csv::binary< header > > m_binary_no_timestamp; // timestamp will not be set
        std::vector< char > m_buffer;
        bool m_headerOnly;
        header m_header; /// default header
};

} }  // namespace snark{ namespace cv_mat {
    
namespace comma { namespace visiting {

template <> struct traits< snark::cv_mat::serialization::header >
{
    template < typename K, typename V >
    static void visit( const K&, snark::cv_mat::serialization::header& h, V& v )
    {
        v.apply( "t", h.timestamp );
        v.apply( "rows", h.rows );
        v.apply( "cols", h.cols );
        v.apply( "type", h.type );
        v.apply( "size", h.size );
    }

    template < typename K, typename V >
    static void visit( const K&, const snark::cv_mat::serialization::header& h, V& v )
    {
        v.apply( "t", h.timestamp );
        v.apply( "rows", h.rows );
        v.apply( "cols", h.cols );
        v.apply( "type", h.type );
        v.apply( "size", h.size );
    }
};

template <> struct traits< snark::cv_mat::serialization::options >
{
    template < typename K, typename V >
    static void visit( const K&, snark::cv_mat::serialization::options& h, V& v )
    {
        v.apply( "fields", h.fields );
        std::string s = h.format.string();
        v.apply( "binary", s );
        h.format = comma::csv::format(s);
        v.apply( "rows", h.rows );
        v.apply( "cols", h.cols );
        v.apply( "type", h.type );
        v.apply( "no-header", h.no_header );
        v.apply( "header-only", h.header_only );
    }

    template < typename K, typename V >
    static void visit( const K&, const snark::cv_mat::serialization::options& h, V& v )
    {
        v.apply( "fields", h.fields );
        v.apply( "binary", h.format.string() );
        v.apply( "rows", h.rows );
        v.apply( "cols", h.cols );
        v.apply( "type", h.type );
        v.apply( "no-header", h.no_header );
        v.apply( "header-only", h.header_only );
    }
};
    
} } // namespace comma { namespace visiting {

