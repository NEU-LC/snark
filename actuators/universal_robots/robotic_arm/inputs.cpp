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
// 3. All advertising materials mentioning features or use of this software
//    must display the following acknowledgement:
//    This product includes software developed by the The University of Sydney.
// 4. Neither the name of the The University of Sydney nor the
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


#include "inputs.h"
#include <string>
#include <iostream>
#include <boost/log/common.hpp>
#include <comma/string/split.h>
#include <comma/string/string.h>

namespace snark { namespace ur { namespace robotic_arm {
    
inputs::inputs( char rover_id_ ) 
        : rover_id( rover_id_ ), buffer_( MAX_BUFFER, '\0' ), mutable_buffer_( &buffer_[0], MAX_BUFFER-1 )
{
    io_.async_receive( mutable_buffer_, boost::posix_time::pos_infin );
}


void inputs::read()
{
    io_.ioservice().poll();
    if( io_.code_would_block() ) { return; }

    if( !io_.code_success() ) { 

        std::ostringstream oss;
        oss << "stdin receive failed with error: " << io_.error_code().value() 
            << " category: " << io_.error_code().category().name() << " - " << io_.error_code().message() << std::endl;
        COMMA_THROW( comma::exception, oss.str() ); 
    }

    // std::cerr << "got sommething: " << io_.received_length() << std::endl;
    typedef std::vector< std::string > vector_s;

    std::string data = comma::strip( buffer_.substr( 0, io_.received_length() ), ";\n" );
    vector_s lines = comma::split( data, '\n' );

    for( vector_s::const_iterator iline=lines.begin(); iline!=lines.end(); ++iline ) 
    {
        // std::cerr << "line: " << *iline << std::endl;
        if( iline->empty() ) continue;

        // now process received line
        vector_s cmds = comma::split( *iline, ";");
        for( vector_s::const_iterator c=cmds.begin(); c!=cmds.end(); ++c ) 
        { 
            // std::cerr << "cmd: " << *c << std::endl;
            command_tokens command = comma::split( *c, ',' );
            // empty line found, or not enough tokens
            if( command.size() <= name_index ) {
                continue;
            }
            
            comma::uint16 id = 0;
            command[ id_index ] = command[ id_index ].substr(1); // removes '>' at start of string
            try {
                id = boost::lexical_cast< comma::uint16 >( command[ id_index ] );
            }
            catch( boost::bad_lexical_cast& el ) {
                std::cerr << "failed to parse rover ID field: " << command[ id_index ] << " - " << el.what() << std::endl;
                continue; // bad ID, ignore command line
            }
            // Not for this rover
            if( id != rover_id ) { 
                std::cerr << "discarding command because of ID mismatch: " << *iline << " " << int(rover_id) << " to " << id << std::endl;
                continue;
            }
            
            command[ name_index ] = comma::strip( command[ name_index ], '"' );
            if( command[ name_index ] == "ESTOP" || command[ name_index ] == "MICROCONTROLLER" )
            {
                // Throws out previous commands
                my_commands = rover_commands();
            }
            
            my_commands.push( command );
        } 

    }

    // set up listening again
    buffer_.resize( MAX_BUFFER, '\0' );
    mutable_buffer_ = boost::asio::mutable_buffer( &buffer_[0], MAX_BUFFER-1 );
    io_.async_receive( mutable_buffer_, boost::posix_time::pos_infin );


}



} }  } // namespace snark { namespace ur { namespace robotic_arm {
