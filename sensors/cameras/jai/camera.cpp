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

#include <Jai_Factory.h>
#include <boost/array.hpp>
#include <comma/base/exception.h>
#include "camera.h"

namespace snark { namespace jai {

// bool OpenFactoryAndCamera()
// {
//    J_STATUS_TYPE    retval;
//    uint32_t         iSize;
//    uint32_t         iNumDev;
//    bool8_t          bHasChange;
// 
//    // Open factory
//    retval = J_Factory_Open((int8_t*)"" , &m_hFactory);
//    if (retval != J_ST_SUCCESS)
//    {
//       printf("Error: Could not open the factory!\n");
//       PrintErrorcode(retval);
//       return false;
//    }
// 
//    printf("Opened the factory\n");
// 
//    // Update camera list
//    retval = J_Factory_UpdateCameraList(m_hFactory, &bHasChange);
//    if (retval != J_ST_SUCCESS)
//    {
//       printf("Error: Could not update the camera list!\n");
//       PrintErrorcode(retval);
//       return false;
//    }
// 
//    printf("Updated the camera list\n");
// 
//    // Get the number of Cameras
//    retval = J_Factory_GetNumOfCameras(m_hFactory, &iNumDev);
//    if (retval != J_ST_SUCCESS)
//    {
//       printf("Error: Could not get the number of cameras!\n");
//       PrintErrorcode(retval);
//       return false;
//    }
//    if (iNumDev == 0)
//    {
//       printf("Warning: There is no camera!\n");
//       return false;
//    }
// 
//    printf("%d cameras were found\n", iNumDev);
// 
//    // Get camera ID
//    iSize = (uint32_t)sizeof(m_sCameraId);
//    retval = J_Factory_GetCameraIDByIndex(m_hFactory, 0, m_sCameraId, &iSize);
//    if (retval != J_ST_SUCCESS)
//    {
//       printf("Error: Could not get the camera ID!");
//       PrintErrorcode(retval);
//       return false;
//    }
// 
//    printf("Camera ID = %s\n", m_sCameraId);
// 
//    // Open camera
//    retval = J_Camera_Open(m_hFactory, m_sCameraId, &m_hCam);
//    if (retval != J_ST_SUCCESS)
//    {
//       printf("Error: Could not open the camera!\n");
//       PrintErrorcode(retval);
//       return false;
//    }
// 
//    printf("Opened the camera\n");
// 
//    return true;
// }

static const char* error_to_string( J_STATUS_TYPE r )
{
   switch( r )
   {
      case J_ST_SUCCESS: return "ok";
      case J_ST_INVALID_BUFFER_SIZE: return "invalid buffer size";
      case J_ST_INVALID_HANDLE: return "invalid handle";
      case J_ST_INVALID_ID: return "invalid id";
      case J_ST_ACCESS_DENIED: return "access denied";
      case J_ST_NO_DATA: return "no data";
      case J_ST_ERROR: return "error";
      case J_ST_INVALID_PARAMETER: return "invalid parameter";
      case J_ST_TIMEOUT: return "timeout";
      case J_ST_INVALID_FILENAME: return "invalid filename";
      case J_ST_INVALID_ADDRESS: return "invalid address";
      case J_ST_FILE_IO: return "file i/o error";
      case J_ST_GC_ERROR: return "genicam error";
      default: return "unknown error code";
   }
}

struct jai::camera::impl
{
    std::string id;
    CAM_HANDLE handle;
    
    impl() : id( J_CAMERA_ID_SIZE, ' ' )
    {
        // todo
    }
    
    std::pair< boost::posix_time::ptime, cv::Mat > read()
    {
        // todo
        return std::pair< boost::posix_time::ptime, cv::Mat >();
    }
    
    void close()
    { 
        // todo
    }

    bool closed() const
    { 
        // todo
        return true;
    }
    
    unsigned long total_bytes_per_frame() const
    {
        // todo
        return 0;
    }
    
    void set( const jai::camera::attributes_type& attributes )
    { 
        // todo
    }
    
    jai::camera::attributes_type attributes() const
    { 
        // todo
        return jai::camera::attributes_type();
    }
};

struct factory::impl
{
    FACTORY_HANDLE handle;
    
    impl()
    {
        J_STATUS_TYPE r = J_Factory_Open( ( int8_t* )( "" ), &handle );
        if( r != J_ST_SUCCESS ) { COMMA_THROW( comma::exception, "failed to create jai camera factory: " << error_to_string( r ) << " (error " << r << ")" ); }
    }
    
    std::vector< std::string > list_devices()
    {
        bool8_t has_change;
        uint32_t number_of_devices;
        J_STATUS_TYPE r = J_Factory_UpdateCameraList( handle, &has_change );
        if( r != J_ST_SUCCESS ) { COMMA_THROW( comma::exception, "failed to update camera list: " << error_to_string( r ) << " (error " << r << ")" ); }
        r = J_Factory_GetNumOfCameras( handle, &number_of_devices );
        if( r != J_ST_SUCCESS ) { COMMA_THROW( comma::exception, "failed to get number of devices: " << error_to_string( r ) << " (error " << r << ")" ); }
        std::vector< std::string > ids( number_of_devices );
        uint32_t size = J_CAMERA_ID_SIZE;
        boost::array< int8_t, J_CAMERA_ID_SIZE > id;
        for( unsigned int i = 0; i < ids.size(); ++i )
        {            
            r = J_Factory_GetCameraIDByIndex( handle, i, &id[0], &size );
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
        CAM_HANDLE h;
        J_STATUS_TYPE r = J_Camera_Open( handle, &id[0], &h );
        if( r != J_ST_SUCCESS ) { COMMA_THROW( comma::exception, "failed to make camera: " << error_to_string( r ) << " (error " << r << ")" ); }
        camera* c = new camera;
        c->pimpl_->handle = h;
        c->pimpl_->id = s;
        return c;
    }
};

jai::factory::factory() : pimpl_( new jai::factory::impl ) {}

jai::factory::~factory() { delete pimpl_; }

std::vector< std::string > jai::factory::list_devices() { return pimpl_->list_devices(); }

camera* jai::factory::make_camera( const std::string& id ) { return pimpl_->make_camera( id ); }

jai::camera::camera() : pimpl_( new jai::camera::impl ) {}

jai::camera::~camera() { delete pimpl_; }

std::pair< boost::posix_time::ptime, cv::Mat > jai::camera::read() { return pimpl_->read(); }

void jai::camera::close() { pimpl_->close(); }

bool jai::camera::closed() const { return pimpl_->closed(); }

//std::vector< XDeviceInformation > jai::camera::list_cameras() { return jai::camera::impl::list_cameras(); }

unsigned long jai::camera::total_bytes_per_frame() const { return pimpl_->total_bytes_per_frame(); }

jai::camera::attributes_type jai::camera::attributes() const { return pimpl_->attributes(); }

void jai::camera::set(const jai::camera::attributes_type& attributes ) { pimpl_->set( attributes ); }

} } // namespace snark { namespace jai {
