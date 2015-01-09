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


/// @author Vsevolod Vlaskine, Cedric Wohlleber

#ifndef WIN32
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>
#endif
#include <deque>
#include <iostream>
#include <cmath>
#include <sstream>
#include <boost/bind.hpp>
#include <comma/base/exception.h>
#include <comma/base/types.h>
#include <comma/csv/stream.h>
#include <comma/io/select.h>
#include "./CameraReader.h"

namespace snark { namespace graphics { namespace View {

CameraReader::CameraReader( comma::csv::options& options )
    : options( options )
    , m_shutdown( false )
    , m_istream( options.filename, options.binary() ? comma::io::mode::binary : comma::io::mode::ascii )
{
}

void CameraReader::shutdown()
{
    if( m_shutdown ) { return; }
    m_shutdown = true;
    if( m_thread ) { m_thread->join(); }
    m_istream.close();
}

bool CameraReader::isShutdown() const { return m_shutdown; }

void CameraReader::read()
{
    while( !m_shutdown && read_once() );
    std::cerr << "view-points: end of camera stream " << options.filename << std::endl;
    m_shutdown = true;
}

void CameraReader::start()
{
    m_thread.reset( new boost::thread( boost::bind( &CameraReader::read, boost::ref( *this ) ) ) );
}

bool CameraReader::read_once()
{
    try
    {
        if( !m_stream ) // quick and dirty: handle named pipes
        {
            if( !m_istream() ) { return true; }
            m_stream.reset( new comma::csv::input_stream< point_with_orientation >( *m_istream, options ) );
        }
        const point_with_orientation* p = m_stream->read();
        if( p == NULL ) { m_shutdown = true; return false; }
        boost::recursive_mutex::scoped_lock lock( m_mutex );
        m_position = p->point;
        m_orientation = p->orientation;
        return true;
    }
    catch( std::exception& ex ) { std::cerr << "view-points: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "view-points: unknown exception" << std::endl; }
    return false;
}

Eigen::Vector3d CameraReader::position() const
{
    boost::recursive_mutex::scoped_lock lock( m_mutex );
    return m_position;
};

Eigen::Vector3d CameraReader::orientation() const
{
    boost::recursive_mutex::scoped_lock lock( m_mutex );
    return m_orientation;
};

} } } // namespace snark { namespace graphics { namespace View {
