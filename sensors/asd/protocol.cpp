
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
        COMMA_THROW(comma::exception, "asd reply error: " << header.error() )
}

} }//namespace snark { namespace asd {
    
