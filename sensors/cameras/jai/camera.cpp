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
#include <boost/regex.hpp>
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
        , height( get< unsigned int >( "Height" ) )
    {
    }
    
    ~impl() { close(); }
    
    void start_acquisition()
    {
        NODE_HANDLE node;
        J_Camera_GetNodeByName( device, ( int8_t * )"AcquisitionStart", &node );
        J_Node_ExecuteCommand( node );
    }
    
    void close()
    {
        if( !device ) { return; }
        NODE_HANDLE node;
        J_Camera_GetNodeByName( device, ( int8_t * )"AcquisitionStop", &node );
        J_Node_ExecuteCommand( node );
        J_Camera_Close( device );
        device = NULL;
    }

    bool closed() const { return device == NULL; }
    
    template < typename T > T get( const char* what ) const
    {
        NODE_HANDLE node;
        validate( std::string( "getting node " ) + what, J_Camera_GetNodeByName( device, ( int8_t * )what, &node ) );
        typename value_traits< T >::internal_type value;
        validate( what, J_Node_GetValueInt64( node, false, &value ) );
        return value;
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
        if( s.size() > J_CAMERA_ID_SIZE ) { COMMA_THROW( comma::exception, "expected id of size not greater than " << J_CAMERA_ID_SIZE << "; got: \"" << s << "\"" ); }
        const std::vector< std::string >& v = list_devices();
        if( v.empty() ) { COMMA_THROW( comma::exception, "no cameras found" ); }
        unsigned int index = 0;
        for( boost::regex regex( s ); index < v.size() && !boost::regex_search( v[index], regex ); ++index );
        if( index == v.size() ) { COMMA_THROW( comma::exception, "no cameras found with id matching \"" << s << "\"" ); }
        boost::array< int8_t, J_CAMERA_ID_SIZE > id;
        ::memcpy( &id[0], &v[index][0], J_CAMERA_ID_SIZE ); // sigh...
        CAM_HANDLE device;
        validate( "making camera", J_Camera_Open( handle, &id[0], &device ) );
        return new camera( new camera::impl( device, v[index] ) );
    }
    
    std::string camera_info( int8_t* id, J_CAMERA_INFO what ) const
    {
        boost::array< int8_t, J_CAMERA_INFO_SIZE > buf;
        uint32_t size = J_CAMERA_INFO_SIZE;
        validate( "getting camera info", J_Factory_GetCameraInfo( handle, &id[0], what, &buf[0], &size ) );
        return std::string( ( const char* )( &buf[0] ), size );
    }
    
    camera::info camera_info( const std::string& s ) const
    {
        boost::array< int8_t, J_CAMERA_ID_SIZE > id;
        ::memcpy( &id[0], &s[0], J_CAMERA_ID_SIZE ); // sigh... otherwise we would need to do const cast
        camera::info info;
        info.manufacturer = camera_info( &id[0], CAM_INFO_MANUFACTURER );
        info.model_name = camera_info( &id[0], CAM_INFO_MODELNAME );
        info.ip_address = camera_info( &id[0], CAM_INFO_IP );
        info.mac_address = camera_info( &id[0], CAM_INFO_MAC );
        info.serial_number = camera_info( &id[0], CAM_INFO_SERIALNUMBER );
        info.username = camera_info( &id[0], CAM_INFO_USERNAME );
        //info.interface_id = camera_info( &id[0], CAM_INFO_INTERFACE_ID );
        return info;
    }
};

camera::settings::settings( camera& c ) : camera_( c.handle() ) {}

void camera::settings::save( const std::string& filename, camera::settings::save_t which ) const
{
    jai::validate( "saving settings to file", J_Camera_SaveSettings( camera_, ( int8_t* )&filename[0], ( _J_SAVE_SETTINGS_FLAG )( which ) ) );
}

void camera::settings::load( const std::string& filename, bool force )
{
    jai::validate( "loading settings from file", J_Camera_LoadSettings( camera_, ( int8_t* )&filename[0], force ? LOAD_FORCE_WRITE : LOAD_AUTO ) );
}

std::string camera::settings::validate( const std::string& filename ) const
{
    J_STATUS_TYPE r = J_Camera_LoadSettings( camera_, ( int8_t* )&filename[0], LOAD_VALIDATE_ONLY );
    switch( r )
    {
        case J_ST_SUCCESS:
            return std::string();
        case J_ST_VALIDATION_ERROR:
        case J_ST_VALIDATION_WARNING:
        {
            uint32_t size = 0;
            jai::validate( "getting camera settings validation error info", J_Camera_GetSettingsValidationErrorInfo( camera_, NULL, &size ) );
            std::string s( std::size_t( size ), ' ' );
            J_Camera_GetSettingsValidationErrorInfo( camera_, ( int8_t* )&s[0], &size );
            return s;
        }
        default:
            COMMA_THROW( comma::exception, "camera settings validation failed: " << error_to_string( r ) );
    }
}

jai::factory::factory() : pimpl_( new jai::factory::impl ) {}

jai::factory::~factory() { delete pimpl_; }

std::vector< std::string > jai::factory::list_devices() { return pimpl_->list_devices(); }

camera* jai::factory::make_camera( const std::string& id ) { return pimpl_->make_camera( id ); }

jai::camera::info jai::factory::camera_info( const std::string& id ) const { return pimpl_->camera_info( id ); }

jai::camera::camera( jai::camera::impl* i ) : pimpl_( i ) {}

jai::camera::~camera() { if( pimpl_ ) { delete pimpl_; } }

void jai::camera::start_acquisition() { pimpl_->start_acquisition(); }

unsigned int jai::camera::width() const { return pimpl_->width; }

unsigned int jai::camera::height() const { return pimpl_->height; }

void jai::camera::close() { pimpl_->close(); }

bool jai::camera::closed() const { return pimpl_->closed(); }

CAM_HANDLE jai::camera::handle() { return pimpl_->device; }

CAM_HANDLE jai::camera::handle() const { return pimpl_->device; }

} } // namespace snark { namespace jai {
