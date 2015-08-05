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

#include <Jai_Factory.h>
#include <comma/base/exception.h>
#include <comma/base/types.h>
#include "error.h"

namespace snark { namespace jai {

const char* error_to_string( J_STATUS_TYPE r )
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

void validate( J_STATUS_TYPE r, const std::string& what )
{
    if( r != J_ST_SUCCESS ) { COMMA_THROW( comma::exception, what << " failed: " << error_to_string( r ) << " (error code: " << r << ")" ); }
}

void validate( const std::string& what, J_STATUS_TYPE r ) { validate( r, what ); }

} } // namespace snark { namespace jai {
