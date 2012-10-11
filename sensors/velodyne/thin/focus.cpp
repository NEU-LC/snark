#include <cassert>
#include <cmath>
#include <comma/base/exception.h>
#include <comma/math/compare.h>
#include "./focus.h"

namespace snark {  namespace velodyne { namespace thin {

focus::focus( double rate, double ratio ) // todo: unit test
    : m_rate( rate )
    , m_ratio( ratio )
    , m_rate_in_focus( 0 )
    , m_rate_out_of_focus( rate )
{
}

double focus::rate_in_focus() const { return m_rate_in_focus; }

double focus::rate_out_of_focus() const { return m_rate_out_of_focus; }

double focus::coverage() const
{
    double c = 0;
    for( Map::const_iterator it = m_regions.begin(); it != m_regions.end(); ++it ) { c += it->second->coverage(); }
    return c;
}

void focus::update()
{
    double c = coverage();
    m_rate_in_focus = m_rate * m_ratio / c;
    if( comma::math::less( 1.0, m_rate_in_focus ) ) { m_rate_in_focus = 1.0; }
    m_rate_out_of_focus = comma::math::equal( m_ratio, 1.0 ) ? 0.0 : m_rate * ( 1 - m_ratio ) / ( 1 - c );
}

void focus::insert( std::size_t id, region* r )
{
    m_regions[id] = boost::shared_ptr< region >( r );
    update();
}

void focus::erase( std::size_t id )
{
    m_regions.erase( id );
    update();
}

} } } // namespace snark {  namespace velodyne { namespace thin {
