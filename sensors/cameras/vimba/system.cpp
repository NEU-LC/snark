// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2016 The University of Sydney
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

#include <comma/base/exception.h>
#include "error.h"
#include "system.h"

namespace snark { namespace vimba {

AVT::VmbAPI::VimbaSystem& system::instance = AVT::VmbAPI::VimbaSystem::GetInstance();

system::system()
{
    VmbErrorType status = instance.Startup();
    if( status != VmbErrorSuccess ) {
        COMMA_THROW( comma::exception, error_msg( "Failed to start API", status ));
    }
}

VmbVersionInfo_t system::version()
{
    VmbVersionInfo_t version;
    VmbErrorType status = instance.QueryVersion( version );
    if( status == VmbErrorSuccess ) { return version; }
    COMMA_THROW( comma::exception, error_msg( "QueryVersion() failed", status ));
}

AVT::VmbAPI::CameraPtrVector system::cameras()
{
    AVT::VmbAPI::CameraPtrVector cameras;
    VmbErrorType status = system::instance.GetCameras( cameras ); // Fetch all cameras
    if( status == VmbErrorSuccess ) { return cameras; }
    COMMA_THROW( comma::exception, error_msg( "GetCameras() failed", status ));
}

AVT::VmbAPI::CameraPtr system::open_first_camera()
{
    AVT::VmbAPI::CameraPtrVector c = cameras();
    if( !c.empty() )
    {
        AVT::VmbAPI::CameraPtr camera = c[0];
        VmbErrorType status = camera->Open( VmbAccessModeFull );
        if( status == VmbErrorSuccess )
        {
            return camera;
        }
        else
        {
            COMMA_THROW( comma::exception, error_msg( "camera::Open() failed", status ));
        }
    }
    else
    {
        COMMA_THROW( comma::exception, "No cameras found" );
    }
}

AVT::VmbAPI::CameraPtr system::open_camera( const std::string& id )
{
    AVT::VmbAPI::CameraPtr camera;
    VmbErrorType status = system::instance.OpenCameraByID( id.c_str(), VmbAccessModeFull, camera );
    if( status == VmbErrorSuccess ) { return camera; }
    COMMA_THROW( comma::exception, error_msg( "OpenCameraById() failed", status ));
}

} } // namespace snark { namespace vimba {
