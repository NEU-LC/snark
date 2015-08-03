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

/// @author vsevolod vlaskine

#include <boost/array.hpp>
#include <comma/base/exception.h>
#include <comma/base/types.h>
#include "camera.h"
#include "error.h"

namespace snark { namespace jai {

template < typename T > struct value_traits {};
template <> struct value_traits < comma::uint32 > { typedef int64_t internal_type; };
template <> struct value_traits < comma::int32 > { typedef int64_t internal_type; };
template <> struct value_traits < comma::int64 > { typedef int64_t internal_type; };

struct jai::camera::impl
{
    mutable CAM_HANDLE device;
    std::string id;
    unsigned int width;
    unsigned int height;
    
    impl( CAM_HANDLE device, const std::string& id )
        : device( device )
        , id( id )
        , width( get< unsigned int >( "Width" ) )
        , height( get< unsigned int >( "height" ) )
    {
    }
    
    ~impl() { close(); }
    
    void close()
    {
        if( !device ) { return; }
        J_Camera_Close( device );
        device = NULL;
    }

    bool closed() const { return device == NULL; }
    
    template < typename T > T get( const char* what ) const
    {
        NODE_HANDLE node;
        validate( "getting node", J_Camera_GetNodeByName( device, ( int8_t * )what, &node ) );
        typename value_traits< T >::internal_type value;
        validate( what, J_Node_GetValueInt64( node, false, &value ) );
        return value;
    }
    
    unsigned long total_bytes_per_frame() const { return width * height * J_MAX_BPP; } // todo: debug
    
    void set( const jai::camera::attributes_type& attributes )
    { 
        COMMA_THROW( comma::exception, "todo" );
    }
    
    jai::camera::attributes_type attributes() const
    {
        COMMA_THROW( comma::exception, "todo" );
    }    
};

struct factory::impl
{
    FACTORY_HANDLE handle;
    
    impl() { validate( "creating camera factory", J_Factory_Open( ( int8_t* )( "" ), &handle ) ); }
    
    ~impl() { J_Factory_Close( handle ); }
    
    std::vector< std::string > list_devices()
    {
        bool8_t has_change;
        uint32_t number_of_devices;
        validate( "updating camera list", J_Factory_UpdateCameraList( handle, &has_change ) );
        validate( "getting number of devices", J_Factory_GetNumOfCameras( handle, &number_of_devices ) );
        std::vector< std::string > ids( number_of_devices );
        uint32_t size = J_CAMERA_ID_SIZE;
        boost::array< int8_t, J_CAMERA_ID_SIZE > id;
        for( unsigned int i = 0; i < ids.size(); ++i )
        {            
            J_STATUS_TYPE r = J_Factory_GetCameraIDByIndex( handle, i, &id[0], &size );
            if( r != J_ST_SUCCESS ) { COMMA_THROW( comma::exception, "failed to get device id for " << i << " camera of " << number_of_devices << " (numbered from 0): " << error_to_string( r ) << " (error " << r << ")" ); }
            ids[i].resize( J_CAMERA_ID_SIZE );
            ::memcpy( &ids[i][0], &id[0], J_CAMERA_ID_SIZE ); // sigh...
        }
        return ids;
    }
    
    jai::camera* make_camera( const std::string& s )
    {
        boost::array< int8_t, J_CAMERA_ID_SIZE > id;
        if( id.empty() )
        {
            const std::vector< std::string >& v = list_devices();
            if( v.empty() ) { COMMA_THROW( comma::exception, "no jai cameras found" ); }
            ::memcpy( &id[0], &v[0][0], J_CAMERA_ID_SIZE );
        }
        else if( s.size() <= J_CAMERA_ID_SIZE )
        {
            ::memcpy( &id[0], &s[0], s.size() );
        }
        else
        {
            COMMA_THROW( comma::exception, "expected id of size not greater than " << J_CAMERA_ID_SIZE << "; got: " << id.size() );
        }        
        CAM_HANDLE device;
        validate( "making camera", J_Camera_Open( handle, &id[0], &device ) );
        return new camera( new camera::impl( device, s ) );
    }
};

jai::factory::factory() : pimpl_( new jai::factory::impl ) {}

jai::factory::~factory() { delete pimpl_; }

std::vector< std::string > jai::factory::list_devices() { return pimpl_->list_devices(); }

camera* jai::factory::make_camera( const std::string& id ) { return pimpl_->make_camera( id ); }

jai::camera::camera( jai::camera::impl* i ) : pimpl_( i ) {}

jai::camera::~camera() { if( pimpl_ ) { delete pimpl_; } }

unsigned int jai::camera::width() const { return pimpl_->width; }

unsigned int jai::camera::height() const { return pimpl_->height; }

void jai::camera::close() { pimpl_->close(); }

bool jai::camera::closed() const { return pimpl_->closed(); }

unsigned long jai::camera::total_bytes_per_frame() const { return pimpl_->total_bytes_per_frame(); }

jai::camera::attributes_type jai::camera::attributes() const { return pimpl_->attributes(); }

void jai::camera::set(const jai::camera::attributes_type& attributes ) { pimpl_->set( attributes ); }

CAM_HANDLE jai::camera::handle() { return pimpl_->device; }

CAM_HANDLE jai::camera::handle() const { return pimpl_->device; }

} } // namespace snark { namespace jai {
