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

#ifndef SNARK_SENSORS_VIMBA_ATTRIBUTE_H_
#define SNARK_SENSORS_VIMBA_ATTRIBUTE_H_

#include <string>
#include <vector>
#include <VimbaCPP/Include/Feature.h>

namespace snark { namespace vimba {

class attribute
{
    public:
        attribute( AVT::VmbAPI::FeaturePtr feature );
        attribute( const attribute& attribute );

        const std::string& name() const { return name_; }
        VmbFeatureDataType type() const { return type_; }
        const std::string& description() const { return description_; }
        const std::vector< std::string >& allowed_values() const { return allowed_values_; }

        unsigned int int_value() const { return value_.int_value; }

        const std::string& value_as_string() const { return value_as_string_; }
        const char*        type_as_string() const;
        std::string        allowed_values_as_string() const;

        void set( const std::string& value );

    private:
        void init_value();
        void init_allowed_values();

        AVT::VmbAPI::FeaturePtr feature_;
        std::string name_;
        VmbFeatureDataType type_;

        // Non-string types
        union value
        {
            VmbInt64_t enum_value;
            VmbBool_t  bool_value;
            double     float_value;
            VmbInt64_t int_value;
        } value_;

        std::string value_as_string_;      // value as string
        std::string description_;
        std::vector< std::string > allowed_values_;
};

} } // namespace snark { namespace vimba {

#endif // SNARK_SENSORS_VIMBA_ATTRIBUTE_H_
