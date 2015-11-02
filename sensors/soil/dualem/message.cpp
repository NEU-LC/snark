#include "message.h"
#include <boost/lexical_cast.hpp>
#include <snark/math/angle.h>

#include <iostream>

namespace snark { namespace dualem { namespace message {
    
inline float degree_to_radian(float degree) { return snark::math::radians(snark::math::degrees(degree)).value; }
pdlmn::data pdlmn::parse(const std::vector<std::string>& text)
{
    pdlmn::data data;
    data.header=text[0];
    data.time=text[1];
    data.hcp_conductivity=boost::lexical_cast<float>(text[2]);
    data.hcp_inphase=boost::lexical_cast<float>(text[3]);
    data.prp_conductivity=boost::lexical_cast<float>(text[4]);
    data.prp_inphase=boost::lexical_cast<float>(text[5]);
    std::cerr<<"pdlmn::parse "<<data.hcp_conductivity<<std::endl;
    return data;
}

pdlma::data pdlma::parse(const std::vector< std::string >& text)
{
    pdlma::data data;
    data.header=text[0];
    data.voltage=boost::lexical_cast<float>(text[1]);
    data.termprature=boost::lexical_cast<float>(text[2]);
    data.pitch=degree_to_radian(boost::lexical_cast<float>(text[3]));
    data.roll=degree_to_radian(boost::lexical_cast<float>(text[4]));
    return data;
}


} } } // namespace snark { namespace dualem { namespace message {
    
