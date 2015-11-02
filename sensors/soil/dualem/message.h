#pragma once
#include <string>
#include <vector>

namespace snark { namespace dualem { namespace message {

struct pdlmn
{
    static bool match(const std::string& id) { return id=="PDLM1" || id=="PDLM2"; }
    struct data
    {
        std::string header;
        std::string time;
        float hcp_conductivity; //mS/m
        float hcp_inphase;  //ppt
        float prp_conductivity; //mS/m
        float prp_inphase;  //ppt
    };
    static data parse(const std::vector<std::string>& text);
};

struct pdlma
{
    static bool match(const std::string& id) { return id=="PDLMA"; }
    struct data
    {
        std::string header;
        float voltage;  //v
        float termprature;  //C
        float pitch;    //radian
        float roll; //radian
    };
    static data parse(const std::vector<std::string>& text);
};

/* do we need this?
struct gpgsv
{
    //satellite visible
    static std::string id() { return "$GPGSV"; }
};
*/

struct gpgga
{
    static std::string id() { return "$GPGGA"; }
    //gpgga(const std::vector<std::string>& text);
};

} } } // namespace snark { namespace dualem { namespace message {
    
