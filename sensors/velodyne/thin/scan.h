#ifndef SNARK_SENSORS_VELODYNE_THIN_SCAN
#define SNARK_SENSORS_VELODYNE_THIN_SCAN

#include <snark/sensors/velodyne/packet.h>

namespace snark {  namespace velodyne { namespace thin {

class scan
{
public:
    scan();
    void thin( velodyne::packet& packet, double rate, double angularSpeed );
    bool empty() const { return m_empty; }
private:
    enum { m_size = 12 * 32 };
    struct index 
    {
        unsigned int idx;
        unsigned int block;
        unsigned int laser;
        index() : idx( 0 ), block( 0 ), laser( 0 ) {}
        const index& operator++()
        {
            ++idx;
            if( block & 0x1 )
            {
                ++laser;
                if( laser < 32 ) { --block; } else { laser = 0; ++block; }
            }
            else
            {
                ++block;
            }
            return *this;
        }
        bool operator==( const index& rhs ) const { return idx == rhs.idx; }
    };
    unsigned int m_scan;
    unsigned int m_count;
    bool m_closed;
    unsigned int m_output;
    bool m_outputCurrentscan;
    bool m_empty; /// true if the last packet is beeing entirely dropped
};


} } } // namespace snark {  namespace velodyne { namespace thin {

#endif // SNARK_SENSORS_VELODYNE_THIN_SCAN

