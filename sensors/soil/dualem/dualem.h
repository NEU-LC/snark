#pragma once
#include <iostream>
#include "message.h"

namespace snark { namespace dualem {
    
class dualem
{
public:
    //dualem(std::istream& is);
    bool process_message(std::string line);
    static std::string check_sum(std::string s);
};

} } //namespace snark { namespace dualem {
