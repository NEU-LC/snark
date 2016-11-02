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


#include <sstream>
#include <comma/application/verbose.h>
#include <comma/base/exception.h>
#include <comma/csv/binary.h>
#include <comma/string/string.h>
#include "serialization.h"

namespace snark{ namespace cv_mat {

serialization::header::header() : rows( 0 ), cols( 0 ), type( 0 ) {}

serialization::header::header( const cv::Mat& m ):
    rows( m.rows ),
    cols( m.cols ),
    type( m.type() ),
    size( m.dataend - m.datastart )
{        
}

serialization::header::header( const std::pair< boost::posix_time::ptime, cv::Mat >& p ) :
    timestamp( p.first ),
    rows( p.second.rows ),
    cols( p.second.cols ),
    type( p.second.type() ),
    size( p.second.dataend - p.second.datastart )
{
}
    

serialization::serialization() :
    m_binary( new comma::csv::binary< header >( comma::csv::format::value< header >(), "", false ) ),
    m_buffer( m_binary->format().size() ),
    m_headerOnly( false )
    
{
}

serialization::serialization( const std::string& fields, const comma::csv::format& format, bool headerOnly, const header& default_header ):
    m_buffer( format.size() ),
    m_headerOnly( headerOnly ),
    m_header( default_header )
{
    if( !fields.empty() ) { m_binary.reset( new comma::csv::binary< header >( format.string(), fields, false, default_header ) ); }
}

serialization::serialization( const serialization::options& options )
{
    if( options.no_header && options.header_only ) { COMMA_THROW( comma::exception, "cannot have no-header and header-only at the same time" ); }
    std::string fields = options.fields.empty() ? std::string( "t,rows,cols,type" ) : options.fields;
    std::vector< std::string > v = comma::split( fields, "," );
    comma::csv::format format;
    for( unsigned int i = 0; i < v.size(); ++i )
    {
        if( v[i] == "t" ) { format += "t"; }
        else { format += "ui"; }
    }
    m_buffer.resize( format.size() );
    m_headerOnly = options.header_only;
    m_header = options.get_header();
    if( !options.no_header ) { m_binary.reset( new comma::csv::binary< header >( format.string(), fields, false, m_header ) ); }
}

std::size_t serialization::put( const std::pair< boost::posix_time::ptime, cv::Mat >& p, char* buf ) const
{
    if( m_binary )
    {
        header h( p );
        m_binary->put( h, buf );
    }
    std::size_t size = p.second.dataend - p.second.datastart;
    ::memcpy( buf + m_binary->format().size(), p.second.datastart, size );
    return m_binary->format().size() + size;
}

std::size_t serialization::get( std::pair< boost::posix_time::ptime, cv::Mat >& p, const char* buf ) const
{
    header h = get_header( buf );
    unsigned int headerSize = 0;
    if( m_binary ) { headerSize = m_binary->format().size(); }
    cv::Mat m( h.rows, h.cols, h.type, const_cast< char* >( buf + headerSize ) );
    m.copyTo( p.second );
    return size( p.second );
}

std::pair< boost::posix_time::ptime, cv::Mat > serialization::get( const char* buf ) const
{
    std::pair< boost::posix_time::ptime, cv::Mat > p;
    get( p, buf );
    return p;
}

serialization::header serialization::get_header( const char* buf ) const
{
    if( m_binary )
    {
        header h;
        m_binary->get( h, buf );
        return h;
    }
    return m_header;
}

std::size_t serialization::size( const cv::Mat& m ) const
{
    unsigned int headerSize = 0;
    if( m_binary ) { headerSize = m_binary->format().size(); }
    return headerSize + ( m.dataend - m.datastart );
}

std::size_t serialization::size( const std::pair< boost::posix_time::ptime, cv::Mat >& m ) const
{
    return size( m.second );
}

std::pair< boost::posix_time::ptime, cv::Mat > serialization::read( std::istream& is )
{
    header h;
    std::pair< boost::posix_time::ptime, cv::Mat > p;
    if( m_binary )
    {
        is.read( &m_buffer[0], m_buffer.size() );
        int count = is.gcount();
        if( count <= 0 ) { return p; }
        if( count < int( m_buffer.size() ) ) { COMMA_THROW( comma::exception, "expected " << m_buffer.size() << " bytes, got " << count ); }        
        m_binary->get( h, &m_buffer[0] );
    }
    else
    {
        h = m_header;
    }
    p.first = h.timestamp;
    try
    {
        p.second = cv::Mat( h.rows, h.cols, h.type );
    }
    catch( cv::Exception& ex )
    {
        std::cerr << comma::verbose.app_name() << ": caught cv::Exception: " << ex.what() << std::endl;
        return std::pair< boost::posix_time::ptime, cv::Mat >();
    }
    std::size_t size = p.second.dataend - p.second.datastart;
    // todo: accumulate
    is.read( const_cast< char* >( reinterpret_cast< const char* >( p.second.datastart ) ), size ); // quick and dirty
    int count = is.gcount();
    return count < int( size ) ? std::pair< boost::posix_time::ptime, cv::Mat >() : p;
}

void serialization::write( std::ostream& os, const std::pair< boost::posix_time::ptime, cv::Mat >& m, bool flush )
{
    if( m_binary )
    {
        header h( m );
        m_binary->put( h, &m_buffer[0] );
        os.write( &m_buffer[0], m_buffer.size() );
    }
    if( !m_headerOnly ) { os.write( reinterpret_cast< const char* >( m.second.datastart ), m.second.dataend - m.second.datastart ); }
    if( flush ) { os.flush(); }
}

unsigned int type_from_string_( const std::string& t )
{
    if( t == "CV_8UC1" || t == "ub" ) { return CV_8UC1; }
    else if( t == "CV_8UC2" || t == "2ub" ) { return CV_8UC2; }
    else if( t == "CV_8UC3" || t == "3ub" ) { return CV_8UC3; }
    else if( t == "CV_8UC4" || t == "4ub" ) { return CV_8UC4; }
    else if( t == "CV_8SC1" || t == "b" ) { return CV_8SC1; }
    else if( t == "CV_8SC2" || t == "2b" ) { return CV_8SC2; }
    else if( t == "CV_8SC3" || t == "3b" ) { return CV_8SC3; }
    else if( t == "CV_8SC4" || t == "4b" ) { return CV_8SC4; }
    else if( t == "CV_16UC1" || t == "uw" ) { return CV_16UC1; }
    else if( t == "CV_16UC2" || t == "2uw" ) { return CV_16UC2; }
    else if( t == "CV_16UC3" || t == "3uw" ) { return CV_16UC3; }
    else if( t == "CV_16UC4" || t == "4uw" ) { return CV_16UC4; }
    else if( t == "CV_16SC1" || t == "w" ) { return CV_16SC1; }
    else if( t == "CV_16SC2" || t == "2w" ) { return CV_16SC2; }
    else if( t == "CV_16SC3" || t == "3w" ) { return CV_16SC3; }
    else if( t == "CV_16SC4" || t == "4w" ) { return CV_16SC4; }
    else if( t == "CV_32SC1" || t == "i" ) { return CV_32SC1; }
    else if( t == "CV_32SC2" || t == "2i" ) { return CV_32SC2; }
    else if( t == "CV_32SC3" || t == "3i" ) { return CV_32SC3; }
    else if( t == "CV_32SC4" || t == "4i" ) { return CV_32SC4; }
    else if( t == "CV_32FC1" || t == "f" ) { return CV_32FC1; }
    else if( t == "CV_32FC2" || t == "2f" ) { return CV_32FC2; }
    else if( t == "CV_32FC3" || t == "3f" ) { return CV_32FC3; }
    else if( t == "CV_32FC4" || t == "4f" ) { return CV_32FC4; }
    else if( t == "CV_64FC1" || t == "d" ) { return CV_64FC1; }
    else if( t == "CV_64FC2" || t == "2d" ) { return CV_64FC2; }
    else if( t == "CV_64FC3" || t == "3d" ) { return CV_64FC3; }
    else if( t == "CV_64FC4" || t == "4d" ) { return CV_64FC4; }
    else { COMMA_THROW( comma::exception, "expected type, got \"" << t << "\"" ); }
}

serialization::header serialization::options::get_header() const
{
    header h;
    h.cols = cols;
    h.rows = rows;
    h.size = cols * rows;
    if( !type.empty() ) { h.type = type_from_string_( type ); }
    return h;
}

std::string serialization::options::usage()
{
    std::stringstream stream;
    stream << "cv::mat serialization options: ';'-separated name=value options" << std::endl;
    stream << "    fields=<fields>; default: t,cols,rows,type" << std::endl;
    stream << "    header-only: if present, output only header (output only)" << std::endl;
    stream << "    no-header: if present, no header" << std::endl;
    stream << "    rows=<rows>: default number of rows (input only)" << std::endl;
    stream << "    cols=<cols>: default number of columns (input only)" << std::endl;
    stream << "    type=<type>: default image type (input only)" << std::endl;
    stream << type_usage();
    return stream.str();
}

std::string serialization::options::type_usage()
{
    std::stringstream stream;
    stream << "    image types (see opencv cv::Mat and cxtypes.hpp):" << std::endl;
    stream << "        CV_8UC1  or ub  (" << CV_8UC1 << ")" << std::endl;
    stream << "        CV_8UC2  or 2ub (" << CV_8UC2 << ")" << std::endl;
    stream << "        CV_8UC3  or 3ub (" << CV_8UC3 << ")" << std::endl;
    stream << "        CV_8UC4  or 4ub (" << CV_8UC4 << ")" << std::endl;
    stream << "        CV_8SC1  or b   (" << CV_8SC1 << ")" << std::endl;
    stream << "        CV_8SC2  or 2b  (" << CV_8SC2 << ")" << std::endl;
    stream << "        CV_8SC3  or 3b  (" << CV_8SC3 << ")" << std::endl;
    stream << "        CV_8SC4  or 4b  (" << CV_8SC4 << ")" << std::endl;
    stream << "        CV_16UC1 or uw  (" << CV_16UC1 << ")" << std::endl;
    stream << "        CV_16UC2 or 2uw (" << CV_16UC2 << ")" << std::endl;
    stream << "        CV_16UC3 or 3uw (" << CV_16UC3 << ")" << std::endl;
    stream << "        CV_16UC4 or 4uw (" << CV_16UC4 << ")" << std::endl;
    stream << "        CV_16SC1 or w   (" << CV_16SC1 << ")" << std::endl;
    stream << "        CV_16SC2 or 2w  (" << CV_16SC2 << ")" << std::endl;
    stream << "        CV_16SC3 or 3w  (" << CV_16SC3 << ")" << std::endl;
    stream << "        CV_16SC4 or 4w  (" << CV_16SC4 << ")" << std::endl;
    stream << "        CV_32SC1 or i   (" << CV_32SC1 << ")" << std::endl;
    stream << "        CV_32SC2 or 2i  (" << CV_32SC2 << ")" << std::endl;
    stream << "        CV_32SC3 or 3i  (" << CV_32SC3 << ")" << std::endl;
    stream << "        CV_32SC4 or 4i  (" << CV_32SC4 << ")" << std::endl;
    stream << "        CV_32FC1 or f   (" << CV_32FC1 << ")" << std::endl;
    stream << "        CV_32FC2 or 2f  (" << CV_32FC2 << ")" << std::endl;
    stream << "        CV_32FC3 or 3f  (" << CV_32FC3 << ")" << std::endl;
    stream << "        CV_32FC4 or 4f  (" << CV_32FC4 << ")" << std::endl;
    stream << "        CV_64FC1 or d   (" << CV_64FC1 << ")" << std::endl;
    stream << "        CV_64FC2 or 2d  (" << CV_64FC2 << ")" << std::endl;
    stream << "        CV_64FC3 or 3d  (" << CV_64FC3 << ")" << std::endl;
    stream << "        CV_64FC4 or 4d  (" << CV_64FC4 << ")" << std::endl;
    return stream.str();
}

} } // namespace snark{ namespace cv_mat {
