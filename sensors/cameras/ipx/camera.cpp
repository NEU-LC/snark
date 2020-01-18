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

#include <sstream>
#include <boost/lexical_cast.hpp>
#include <IpxCameraErr.h>
#include <comma/base/exception.h>
#include "camera.h"

#include <iostream>

namespace snark { namespace ipx {

static const char* access_status_to_string( int status )
{
    switch( status )
    {
        case IpxCam::DeviceInfo::AccessStatusUnknown: return "unknown";
        case IpxCam::DeviceInfo::AccessStatusReadWrite: return "read-write";
        case IpxCam::DeviceInfo::AccessStatusReadOnly: return "read-only";
        case IpxCam::DeviceInfo::AccessStatusNoAccess: return "no access";
        case IpxCam::DeviceInfo::IpSubnetMismatch: return "subnet mismatch";
        default: return "unknown";
    }
}
    
system::system(): system_( IpxCam::IpxCam_GetSystem() )
{
    if( !system_ ) { COMMA_THROW( comma::exception, "failed to create system" ); }
    interface_list_ = system_->GetInterfaceList();
    if( !interface_list_ ) { COMMA_THROW( comma::exception, "failed to get interface list" ); }
    if( interface_list_->GetCount() == 0 ) { COMMA_THROW( comma::exception, "no interfaces available" ); }
}

system::~system()
{ 
    interface_list_->Release();
    system_->Release();
}

std::string system::interfaces_description()
{
    std::ostringstream oss;
    for( auto i = interface_list_->GetFirst(); i; i = interface_list_->GetNext() ) { oss << i->GetDescription() << std::endl; }
    return oss.str();
}

std::string system::devices_description()
{
    std::ostringstream oss;
    for( auto i = interface_list_->GetFirst(); i; i = interface_list_->GetNext() ) 
    {
        i->ReEnumerateDevices( nullptr, 200 );
        ipx::unique_ptr< IpxCam::DeviceInfoList > device_info_list( i->GetDeviceInfoList() );
        for( auto d = device_info_list->GetFirst(); d; d = device_info_list->GetNext() ) { oss << i->GetDescription() << "," << d->GetID() << "," << d->GetDisplayName() << std::endl; }
    }
    return oss.str();
}

IpxCam::DeviceInfo* system::device_info( const std::string& id )
{
    for( auto i = interface_list_->GetFirst(); i; i = interface_list_->GetNext() ) 
    {
        i->ReEnumerateDevices( nullptr, 200 );
        ipx::unique_ptr< IpxCam::DeviceInfoList > device_info_list( i->GetDeviceInfoList() );
        if( device_info_list->GetCount() == 0 ) { break; }
        for( auto d = device_info_list->GetFirst(); d; d = device_info_list->GetNext() ) { if( id.empty() || d->GetID() == id ) { return d; } }
    }
    COMMA_THROW( comma::exception, ( id.empty() ? "no devices available": "device not found, id: \"" + id + "\"" ) );
}
    
camera::camera( IpxCam::DeviceInfo* device_info )
{
    if( device_info->GetAccessStatus() == IpxCam::DeviceInfo::IpSubnetMismatch ) { COMMA_THROW( comma::exception, "cannot connect due to ip subnet mismatch error" ); }
    if( device_info->GetAccessStatus() != IpxCam::DeviceInfo::AccessStatusReadWrite ) { COMMA_THROW( comma::exception, "failed to connect due to access status: " << access_status_to_string( device_info->GetAccessStatus() ) ); }
    device_.reset( IpxCam::IpxCam_CreateDevice( device_info ) );
    if( !device_ ) { COMMA_THROW( comma::exception, "failed to create device" ); }
    id_ = device_info->GetID();
}

std::string camera::get_parameter( const std::string& name )
{
    auto params = device_->GetCameraParameters();
    if( !params ) { COMMA_THROW( comma::exception, "device " << id_ << ": failed to get parameters" ); }
    IpxGenParam::Param *p = params->GetParam( &name[0], nullptr );
    if( !p ) { COMMA_THROW( comma::exception, "device " << id_ << ": parameter not found: '" << p << "'" ); }
    if( !p->IsAvailable() ) { COMMA_THROW( comma::exception, "device " << id_ << ": parameter not available: '" << p << "'" ); }
    if( !p->IsReadable() ) { COMMA_THROW( comma::exception, "device " << id_ << ": parameter not readable: '" << p << "'" ); }
    switch( p->GetType() )
    {
        case IpxGenParam::ParamInt: return boost::lexical_cast< std::string >( dynamic_cast< IpxGenParam::Int* >( p )->GetValue() );
        case IpxGenParam::ParamFloat: return boost::lexical_cast< std::string >( dynamic_cast< IpxGenParam::Float* >( p )->GetValue() );
        case IpxGenParam::ParamString: return dynamic_cast< IpxGenParam::String* >( p )->GetValue();
        case IpxGenParam::ParamEnum: return dynamic_cast< IpxGenParam::Enum* >( p )->GetValueStr();
        case IpxGenParam::ParamBoolean: return boost::lexical_cast< std::string >( dynamic_cast< IpxGenParam::Boolean* >( p )->GetValue() );
        case IpxGenParam::ParamCommand: return boost::lexical_cast< std::string >( dynamic_cast< IpxGenParam::Command* >( p )->IsDone() );
        case IpxGenParam::ParamCategory:
        {
            auto r = dynamic_cast< IpxGenParam::Category* >( p );
            std::string list;
            std::string semicolon;
            for( unsigned int i = 0; i < r->GetCount(); ++i ) { list += semicolon + r->GetParamByIndex( i, nullptr )->GetDisplayName(); semicolon = ";"; }
            return list;
        }
        default: break; // never here
    }
    COMMA_THROW( comma::exception, "device " << id_ << ": on '" << p << "': expected parameter type; got: " << p->GetType() );
}

void camera::set_parameter( const std::string& name, const std::string& value )
{
    auto params = device_->GetCameraParameters();
    if( !params ) { COMMA_THROW( comma::exception, "device " << id_ << ": failed to get parameters" ); }
    IpxGenParam::Param *p = params->GetParam( &name[0], nullptr );
    if( !p ) { COMMA_THROW( comma::exception, "device " << id_ << ": parameter not found: '" << p << "'" ); }
    if( !p->IsAvailable() ) { COMMA_THROW( comma::exception, "device " << id_ << ": parameter not available: '" << p << "'" ); }
    if( !p->IsWritable() ) { COMMA_THROW( comma::exception, "device " << id_ << ": parameter not writable: '" << p << "'" ); }
    
    // todo
}

void camera::connect()
{
    device_->Release();
    //device_info_ = IpxGui::SelectCameraA( ipx::system_, "get device selection" );
    //device_ = IpxCam::IpxCam_CreateDevice( device_info_ );
    //if( !device_ ) { COMMA_THROW( comma::exception, "failed to connect" ); }
    
}

} } // namespace snark { namespace ipx {
