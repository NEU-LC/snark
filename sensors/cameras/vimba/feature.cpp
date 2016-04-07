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

#include <algorithm>
#include <iostream>
#include <sstream>
#include <boost/lexical_cast.hpp>
#include <comma/application/verbose.h>
#include <comma/base/exception.h>
#include "error.h"
#include "feature.h"

namespace snark { namespace vimba {

// Prints out the value of a given feature
std::string feature_value( const AVT::VmbAPI::FeaturePtr& feature )
{
    std::ostringstream value;
    VmbFeatureDataType type;
    VmbErrorType status = feature->GetDataType( type );
    
    if( status != VmbErrorSuccess )
    {
        write_error( "Could not get feature Data Type", status );
    }
    else
    {
        switch( type )
        {
            case VmbFeatureDataEnum:
            case VmbFeatureDataString:
                {
                    std::string string_value;
                    status = feature->GetValue( string_value );
                    if ( status == VmbErrorSuccess )
                    {
                        return string_value;
                    }
                }
                break;
            case VmbFeatureDataBool:
                {
                    VmbBool_t bool_value;
                    status = feature->GetValue( bool_value );
                    if ( status == VmbErrorSuccess )
                    {
                        value << bool_value;
                    }
                }
                break;
            case VmbFeatureDataFloat:
                {
                    double float_value;
                    status = feature->GetValue( float_value );
                    if( status == VmbErrorSuccess)
                    {
                        value << float_value;
                    }
                }
                break;
            case VmbFeatureDataInt:
                {
                    VmbInt64_t int_value;
                    status = feature->GetValue( int_value );
                    if( status == VmbErrorSuccess)
                    {
                        value << int_value;
                    }
                }
                break;
            case VmbFeatureDataCommand:
            default:
                value << "[None]";
                break;
        }
        if( status != VmbErrorSuccess )
        {
            write_error( "Could not get feature value", status );
        }
    }
    return value.str();
}

std::string wrap( std::string text, size_t width = 80, std::string prefix = "")
{
    std::istringstream words( text );
    std::ostringstream wrapped;
    std::string word;

    size_t wrap_width = width - prefix.length();

    if( words >> word )
    {
        wrapped << word;
        size_t space_left = wrap_width - word.length();
        while( words >> word )
        {
            if( space_left < word.length() + 1 ) {
                wrapped << '\n' << prefix << word;
                space_left = wrap_width - word.length();
            } else {
                wrapped << ' ' << word;
                space_left -= word.length() + 1;
            }
        }
    }
    return wrapped.str();
}

void print_feature_entry( std::string label, std::string value )
{
    std::string prefix( label.length() + 2, ' ' );
    std::cout << label << ": " << wrap( value, 80, prefix ) << "\n";
}

// Prints all details of a feature
void print_feature( const AVT::VmbAPI::FeaturePtr& feature, bool verbose )
{
    std::string name;                    // The name of the feature
    std::string description;             // A long description of the feature
    std::string value;                   // The value of the feature

    value = feature_value( feature );

    VmbErrorType status = feature->GetName( name );
    if( status != VmbErrorSuccess )
    {
        write_error( "Could not get feature name", status );
    }

    if( !verbose )
    {
        std::cout << name << "=" << value << std::endl;
    }
    else
    {
        status = feature->GetDescription( description );
        if( status != VmbErrorSuccess )
        {
            write_error( "Could not get feature description", status );
        }
        print_feature_entry( "Name       ", name );
        print_feature_entry( "Value      ", value );
        print_feature_entry( "Description", description );
        std::cout << std::endl;
    }
}

void print_features( AVT::VmbAPI::FeaturePtrVector features, bool verbose )
{
    std::for_each( features.begin(), features.end(),
                 [verbose] ( AVT::VmbAPI::FeaturePtr& f ) { print_feature( f, verbose ); } );
}

VmbErrorType run_command( AVT::VmbAPI::FeaturePtr feature )
{
    VmbErrorType status = feature->RunCommand();
    if( status == VmbErrorSuccess )
    {
        bool is_command_done = false;
        do
        {
            status = feature->IsCommandDone( is_command_done );
            if( status != VmbErrorSuccess )
            {
                break;
            }
        } while( !is_command_done );
    }
    return status;
}

std::string data_type_to_string( VmbFeatureDataType data_type )
{
    switch( data_type )
    {
        case VmbFeatureDataUnknown: return "unknown";
        case VmbFeatureDataInt:     return "integer";
        case VmbFeatureDataFloat:   return "floating point";
        case VmbFeatureDataEnum:    return "enumeration";
        case VmbFeatureDataString:  return "string";
        case VmbFeatureDataBool:    return "boolean";
        case VmbFeatureDataCommand: return "command";
        case VmbFeatureDataRaw:     return "raw";
        case VmbFeatureDataNone:    return "no data";
    }
    return "";                          // Quiet gcc warning
}

void set_feature( AVT::VmbAPI::CameraPtr camera
                , std::string feature_name
                , std::string value )
{
    AVT::VmbAPI::FeaturePtr feature;
    VmbErrorType status = camera->GetFeatureByName( feature_name.c_str(), feature );
    if( status == VmbErrorSuccess )
    {
        VmbFeatureDataType data_type;
        status = feature->GetDataType( data_type );
        if( status == VmbErrorSuccess )
        {
            if( comma::verbose )
            {
                std::cerr << "Setting \"" << feature_name << "\" feature";
                if( !value.empty() )
                {
                    std::cerr << " to " << data_type_to_string( data_type ) << " value \"" << value << "\"";
                }
                std::cerr << std::endl;
            }
            switch( data_type )
            {
                case VmbFeatureDataUnknown:               // Unknown feature type
                    COMMA_THROW( comma::exception, "unknown feature \"" << feature_name << "\"" );
                    break;
                case VmbFeatureDataInt:                   // 64 bit integer feature
                    status = feature->SetValue( boost::lexical_cast< VmbInt64_t >( value ));
                    break;
                case VmbFeatureDataFloat:                 // 64 bit floating point feature
                    status = feature->SetValue( boost::lexical_cast< double >( value ));
                    break;
                case VmbFeatureDataEnum:                  // Enumeration feature
                case VmbFeatureDataString:                // String feature
                    status = feature->SetValue( value.c_str() );
                    break;
                case VmbFeatureDataBool:                  // Boolean feature
                    status = feature->SetValue( boost::lexical_cast< bool >( value ));
                    break;
                case VmbFeatureDataCommand:               // Command feature
                    status = run_command( feature );
                    break;
                case VmbFeatureDataRaw:                   // Raw (direct register access) feature
                    COMMA_THROW( comma::exception, "feature \"" << feature_name << "\" is a direct register - setting this type is unsupported" );
                    break;
                case VmbFeatureDataNone:                  // Feature with no data
                    COMMA_THROW( comma::exception, "feature \"" << feature_name << "\" has no data" );
                    break;
            }
        }
    }
    if( status != VmbErrorSuccess )
    {
        // TODO: offer suggestions for legal values for enumeration or legal ranges for other types
        COMMA_THROW( comma::exception, "failed to set feature \"" << feature_name << " to " << value << ", Error: " << status << ": " << error_code_to_string( status ));
    }
}

} } // namespace snark { namespace vimba {
