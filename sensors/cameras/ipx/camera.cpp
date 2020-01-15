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

#include <IpxCameraErr.h>
#include <comma/base/exception.h>
#include "camera.h"

namespace snark { namespace ipx {

system::system(): system_( IpxCam::IpxCam_GetSystem() )
{ 
    if( !system_ ) { COMMA_THROW( comma::exception, "failed to create system" ); }
    interface_list_ = system_->GetInterfaceList();
    if( !interface_list_ ) { COMMA_THROW( comma::exception, "failed to get interface list" ); }
    if( interface_list_->GetCount() == 0 ) { COMMA_THROW( comma::exception, "no interfaces available" ); }
}

system::~system()
{
    if( interface_list_ ) { interface_list_->Release(); }
    if( system_ ) { system_->Release(); }
}

std::vector< std::string > system::description() const
{
    std::vector< std::string > v( interface_list_->GetCount() );
    for( auto i = interface_list_->GetFirst(); i; i = interface_list_->GetNext() ) { v.push_back( i->GetDescription() ); }
    return v;
}
    
camera::camera( IpxCam::Device* device ): device_( device ) {}

camera::~camera() { device_->Release(); }

void camera::connect()
{
    device_->Release();
    //device_info_ = IpxGui::SelectCameraA( ipx::system_, "get device selection" );
    //device_ = IpxCam::IpxCam_CreateDevice( device_info_ );
    //if( !device_ ) { COMMA_THROW( comma::exception, "failed to connect" ); }
    
}

} } // namespace snark { namespace ipx {
