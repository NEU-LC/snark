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

struct factory
{
    FACTORY_HANDLE handle;
    
    factory()
    {
        if( J_Factory_Open( ( int8_t* )( "" ), &handle ) != J_ST_SUCCESS ) { COMMA_THROW( comma::exception, "failed to create jai camera factory" ); }
    }
};

struct jai::camera::impl
{
    jai::factory factory;
    CAM_HANDLE handle;
    
    impl()
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

jai::camera::camera() : pimpl_( new impl ) {}

jai::camera::~camera() { delete pimpl_; }

std::pair< boost::posix_time::ptime, cv::Mat > jai::camera::read() { return pimpl_->read(); }

void jai::camera::close() { pimpl_->close(); }

bool jai::camera::closed() const { return pimpl_->closed(); }

//std::vector< XDeviceInformation > jai::camera::list_cameras() { return jai::camera::impl::list_cameras(); }

unsigned long jai::camera::total_bytes_per_frame() const { return pimpl_->total_bytes_per_frame(); }

jai::camera::attributes_type jai::camera::attributes() const { return pimpl_->attributes(); }

void jai::camera::set(const jai::camera::attributes_type& attributes ) { pimpl_->set( attributes ); }

} } // namespace snark { namespace jai {
