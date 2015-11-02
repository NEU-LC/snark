#include "dualem.h"
#include <comma/string/split.h>
#include <comma/application/verbose.h>
#include <cstdio>
#include <vector>
#include <string>

#include <comma/name_value/serialize.h>
#include "traits.h"
namespace snark { namespace dualem {
    

bool dualem::process_message(std::string line)
{
    if(line.empty()||line[0]!='$') { return false; }
    std::size_t p=line.find_last_of("*");
    if(p!=std::string::npos)
    {
        //has check sum
        if(check_sum(line.substr(1,p-1)) != line.substr(p + 1))
        {
            std::cerr<< comma::verbose.app_name()<< ": check sum error: original " << line.substr(p + 1) << " calculated " << check_sum(line.substr(1,p-1))<<std::endl;
            std::cerr<<"line "<<line<<std::endl;
            std::cerr<<"left "<<line.substr(1,p-1)<<std::endl;
            std::cerr<<"right "<<line.substr(p + 1)<<std::endl;
            return false;
        }
        line=line.substr(1,p-1);
    }
    std::vector<std::string> text=comma::split(line, ',');
    if(message::pdlmn::match(text[0])) 
    { 
        message::pdlmn::data d=message::pdlmn::parse(text); 
        comma::write_json(d, std::cout);
        return true;
    }
    else if(message::pdlma::match(text[0]))
    { 
        message::pdlma::data d=message::pdlma::parse(text); 
        comma::write_json(d, std::cout);
        return true;
    }
    //else if(text[0]==message::gpgsv::id()) { message::gpgsv d(text); }
    //else if(text[0]==message::gpgga::id()) { message::gpgga d(text); }
    std::cerr<<"no match: "<<line<<std::endl;
    return false;
}

std::string snark::dualem::dualem::check_sum(std::string s)
{
    int cs=0;
    for(std::size_t i=0;i<s.size();i++)
        cs ^= s[i];
    static char buf[12];
    std::sprintf(buf,"%02X",cs);
    return std::string(buf);
}

} } //namespace snark { namespace dualem {

