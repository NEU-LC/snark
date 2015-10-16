#pragma once
#include <comma/io/stream.h>
#include <comma/application/cverbose.h>
#include "commands.h"

namespace snark { namespace asd {

struct protocol
{
    char buf[2000];
    comma::io::iostream ios;
    bool trace;
    protocol(const std::string& address, bool trace);
    template<typename T>
    typename T::reply send(const std::string& command);
    void handle_reply(const commands::reply_header& header);
};

template<typename T>
typename T::reply protocol::send(const std::string& command)
{
    if(trace)
        comma::cverbose<<"-> "<<command<<std::endl;
    *ios<<command<<std::flush;
    typename T::reply reply;
    ios->read(reply.data(),reply.size);
    //error handling
    handle_reply(reply.header);
    return reply;
}

} }//namespace snark { namespace asd {

