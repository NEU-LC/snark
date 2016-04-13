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

#include <comma/application/signal_flag.h>
#include <comma/application/verbose.h>
#include <comma/name_value/map.h>

#include "camera.h"
#include "error.h"
#include "frame_observer.h"
#include "system.h"

namespace snark { namespace vimba {

static const unsigned int num_frames = 3;

camera::camera( const std::string& camera_id )
{
    camera_ = system::open_camera( camera_id );
}

camera::~camera()
{
    if( camera_ ) camera_->Close();
}

void camera::print_info( bool verbose ) const
{
    std::string id;
    std::string name;
    std::string model;
    std::string serial_number;
    std::string interface_id;

    VmbErrorType status = camera_->GetID( id );
    if( status != VmbErrorSuccess ) { write_error( "Could not get camera ID", status ); }
                
    status = camera_->GetName( name );
    if( status != VmbErrorSuccess ) { write_error( "Could not get camera name", status ); }

    status = camera_->GetModel( model );
    if( status != VmbErrorSuccess ) { write_error( "Could not get camera mode name", status ); }

    status = camera_->GetSerialNumber( serial_number );
    if( status != VmbErrorSuccess ) { write_error( "Could not get camera serial number", status ); }

    status = camera_->GetInterfaceID( interface_id );
    if( status != VmbErrorSuccess ) { write_error( "Could not get interface ID", status ); }

    if( verbose )
    {
        std::cout << "\nCamera ID    : " << id
                  << "\nCamera Name  : " << name
                  << "\nModel Name   : " << model
                  << "\nSerial Number: " << serial_number
                  << "\nInterface ID : " << interface_id  << std::endl;
    }
    else
    {
        std::cout << "id=\"" << id << "\",name=\"" << name << "\",serial=\"" << serial_number << "\"" << std::endl;
    }
}

std::vector< attribute > camera::attributes() const
{
    std::vector< attribute > attributes;
    AVT::VmbAPI::FeaturePtrVector features;
    VmbErrorType status = camera_->GetFeatures( features );
    if( status == VmbErrorSuccess )
    {
        for( AVT::VmbAPI::FeaturePtrVector::iterator it = features.begin();
             it != features.end();
             ++it )
        {
            attribute a( *it );
            attributes.push_back( a );
        }
    }
    else
    {
        COMMA_THROW( comma::exception, error_msg( "GetFeatures() failed", status ));
    }
    return attributes;
}

void camera::set_feature( const std::string& feature_name, const std::string& value ) const
{
    AVT::VmbAPI::FeaturePtr feature;
    VmbErrorType status = camera_->GetFeatureByName( feature_name.c_str(), feature );
    if( status == VmbErrorSuccess )
    {
        attribute a( feature );
        a.set( value );
    }
    else
    {
        COMMA_THROW( comma::exception, error_msg( "GetFeatureByName() failed", status ));
    }
}

void camera::set_features( const std::string& name_value_pairs ) const
{
    comma::name_value::map m( name_value_pairs );
    for( comma::name_value::map::map_type::const_iterator it = m.get().begin(); it != m.get().end(); ++it )
    {
        set_feature( it->first, it->second );
    }
}

void camera::capture_images( boost::shared_ptr< snark::cv_mat::serialization > serialization ) const
{
    VmbErrorType status;

    status = start_continuous_image_acquisition( serialization );
    if ( status == VmbErrorSuccess )
    {
        comma::signal_flag is_shutdown;
        do {
            sleep( 1 );
        } while( !is_shutdown );

        stop_continuous_image_acquisition();
    }
}

VmbErrorType camera::start_continuous_image_acquisition( boost::shared_ptr< snark::cv_mat::serialization > serialization ) const
{
    comma::verbose << "Start continuous image acquisition" << std::endl;

    // Create a frame observer for this camera
    // (This will be wrapped in a shared_ptr so we don't delete it)
    frame_observer* fo = new frame_observer( camera_, serialization );
    // Start streaming
    VmbErrorType status = camera_->StartContinuousImageAcquisition( num_frames, AVT::VmbAPI::IFrameObserverPtr( fo ));
    return status;
}

void camera::stop_continuous_image_acquisition() const
{
    comma::verbose << "Stop continuous image acquisition" << std::endl;
    camera_->StopContinuousImageAcquisition();
}

} } // namespace snark { namespace vimba {
