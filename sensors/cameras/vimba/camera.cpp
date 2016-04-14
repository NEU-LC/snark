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

#include <sstream>
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

camera::name_values camera::info() const
{
    name_values name_value_pairs;

    add_name_value( "id",            &AVT::VmbAPI::Camera::GetID,           name_value_pairs );
    add_name_value( "name",          &AVT::VmbAPI::Camera::GetName,         name_value_pairs );
    add_name_value( "model",         &AVT::VmbAPI::Camera::GetModel,        name_value_pairs );
    add_name_value( "serial_number", &AVT::VmbAPI::Camera::GetSerialNumber, name_value_pairs );
    add_name_value( "interface_id",  &AVT::VmbAPI::Camera::GetInterfaceID,  name_value_pairs );

    return name_value_pairs;
}

void camera::add_name_value( const char* label, getter_fn fn, name_values& name_value_pairs ) const
{
    std::string value;
    VmbErrorType status = ( *camera_.*fn )( value );
    if( status == VmbErrorSuccess )
    {
        name_value_pairs[ label ] = value;
    }
    else
    {
        std::ostringstream msg;
        msg << "Count not get " << label;
        write_error( msg.str(), status );
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

void camera::set_feature( const std::string& name, const std::string& value ) const
{
    AVT::VmbAPI::FeaturePtr feature;
    VmbErrorType status = camera_->GetFeatureByName( name.c_str(), feature );
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

void camera::capture_images( std::vector< snark::cv_mat::filter > filters
                           , boost::shared_ptr< snark::cv_mat::serialization > serialization ) const
{
    VmbErrorType status;

    status = start_continuous_image_acquisition( filters, serialization );
    if ( status == VmbErrorSuccess )
    {
        comma::signal_flag is_shutdown;
        do {
            sleep( 1 );
        } while( !is_shutdown );

        stop_continuous_image_acquisition();
    }
}

VmbErrorType camera::start_continuous_image_acquisition( std::vector< snark::cv_mat::filter > filters
                                                       , boost::shared_ptr< snark::cv_mat::serialization > serialization ) const
{
    comma::verbose << "Start continuous image acquisition" << std::endl;

    // Create a frame observer for this camera
    // (This will be wrapped in a shared_ptr so we don't delete it)
    frame_observer* fo = new frame_observer( camera_, filters, serialization );
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
