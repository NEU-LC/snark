// Copyright (c) 2011 The University of Sydney

/// @author Vsevolod Vlaskine, Cedric Wohlleber

#include "camera_reader.h"

namespace snark { namespace graphics { namespace view {

CameraReader::CameraReader( const comma::csv::options& options )
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

void CameraReader::start() { m_thread.reset( new boost::thread( boost::bind( &CameraReader::read, boost::ref( *this ) ) ) ); }

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

} } } // namespace snark { namespace graphics { namespace view {
