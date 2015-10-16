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

#include "protocol.h"
#include <comma/io/publisher.h>

namespace snark { namespace asd {
    

protocol::protocol(const std::string& address, bool t) : ios(address), trace(t)
{
    comma::cverbose<<"asd::protocol"<<std::endl;
//     comma::io::select select;
//     select.read().add( ios->rdbuf()->native() );
//     select.wait(3);
//     if( !select.read().ready( ios->rdbuf()->native() ) )
//         COMMA_THROW( comma::exception, "no reply received from asd device on connection" ); 
    ios->getline(buf,sizeof(buf),'\n');
    if(trace) 
        comma::cverbose<<"<- "<<buf<<std::endl;
    ios->getline(buf,sizeof(buf));
    if(trace) 
        comma::cverbose<<"<- "<<buf<<std::endl;
}

void protocol::handle_reply(const commands::reply_header& header)
{
    if(trace)
        comma::cverbose<<"reply header: "<<header.header()<<" error: "<<header.error()<<std::endl;
    if(header.error() != 0)
        COMMA_THROW(comma::exception, "asd reply error: " << header.error() );
}

} }//namespace snark { namespace asd {
    
