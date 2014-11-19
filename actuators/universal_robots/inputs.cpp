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

namespace snark { namespace ur {
    
inputs::inputs( char robot_id_ ) 
        : robot_id( robot_id_ ), istream_( "-", comma::io::mode::ascii ) {
            select_.read().add( istream_.fd() );
        }


typedef std::vector< std::string > vector_s;

void inputs::read( const boost::posix_time::time_duration& timeout )
{
    if( !istream_->good() ) { COMMA_THROW( comma::exception, "inputs - input stream not good()" ); }

    select_.wait( timeout ); // can be 0 

    if( !select_.read().ready( istream_.fd() ) ) { return; }

    do
    {
        std::string line;
        std::getline( *istream_, line ); 
        // std::cerr << "size: " << istream_->rdbuf()->in_avail() << std::endl;
        // std::cerr << "line: " << line << std::endl;
        if( line.empty() ) continue;

        // now process received line
        vector_s cmds = comma::split( line, ";");
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
                std::cerr << "failed to parse robot ID field: " << command[ id_index ] << " - " << el.what() << std::endl;
                continue; // bad ID, ignore command line
            }
            // Not for this robot
            if( id != robot_id ) { 
                std::cerr << "discarding command because of ID mismatch: " << line << " " << int(robot_id) << " to " << id << std::endl;
                continue;
            }
            
            command[ name_index ] = comma::strip( command[ name_index ], '"' );
            if( command[ name_index ] == "ESTOP" || command[ name_index ] == "MICROCONTROLLER" )
            {
                // Throws out previous commands
                my_commands = robot_commands();
            }
            
            // std::cerr << "pushed line: " << line << std::endl;
            my_commands.push( command );
        } 
        
    } 
    while( istream_->rdbuf()->in_avail() > 0 );
}



} } // namespace snark { namespace ur {
