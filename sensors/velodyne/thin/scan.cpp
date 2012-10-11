
#include "scan.h"
#include <comma/math/compare.h>

namespace snark {  namespace velodyne { namespace thin {

scan::scan():
    m_scan( 0 ),
    m_count( 0 ),
    m_output( 0 ),
    m_outputCurrentscan( true ),
    m_empty( false )
{
}

void scan::thin ( velodyne::packet& packet, double rate, double angularSpeed )
{
    m_empty = true;
    ++m_count;
    for( index i; i.idx < 12 * 32 ; ++i )
    {
        bool valid = packet.blocks[i.block].lasers[i.laser].range() > 0;
        if( !valid ) { continue; }
        if( m_count > 100 && packet.blocks[i.block].rotation() < 500 )
        {
            ++m_scan;
            m_count = 0;
            m_outputCurrentscan = ( m_output == 0 ) || ( m_output < rate * m_scan );
            if( m_outputCurrentscan ) { m_output++; }
        }
        if( m_outputCurrentscan )
        {
            m_empty = false;
        }
        else
        {
            packet.blocks[i.block].lasers[i.laser].range = 0;
        }
    }
}



} } } // namespace snark {  namespace velodyne { namespace thin {
