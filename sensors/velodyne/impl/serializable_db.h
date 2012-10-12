// This file is part of snark, a generic and flexible library
// for robotics research.
//
// Copyright (C) 2011 The University of Sydney
//
// snark is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// snark is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
// for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with snark. If not, see <http://www.gnu.org/licenses/>.

#ifndef SNARK_SENSORS_VELODYNE_IMPL_SERIALIZABLEDB_H_
#define SNARK_SENSORS_VELODYNE_IMPL_SERIALIZABLEDB_H_

//#include <iostream>
#include <string>
#include <vector>
#include <boost/array.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/utility.hpp>
//#include <boost/serialization/list.hpp>
#include <boost/serialization/version.hpp>
#include <boost/static_assert.hpp>
#include <boost/type_traits/is_class.hpp>

namespace snark {  namespace velodyne { namespace impl {

/// an ugly fixture for reverse engineering of db.xml;
/// we would not need it, if we knew the original data type
/// velodyne used. Contact velodyne and make it simple!
template < bool AddDummyversion = false > // to avoid silly Windows warning
struct serializable_array_item
{
    template< class Archive >
    static void add_item_version( Archive& ) {}
};

/// an ugly fixture for reverse engineering of db.xml;
/// we would not need it, if we knew the original data type
/// velodyne used. Contact velodyne and make it simple!
template <>
struct serializable_array_item< true >
{ 
    template< class Archive >
    static void add_item_version( Archive& ar )
    {
        int itemversion( 0 );
        ar & boost::serialization::make_nvp( "item_version", itemversion );
    }
};

/// an ugly fixture for reverse engineering of db.xml;
/// we would not need it, if we knew the original data type
/// velodyne used. Contact velodyne and make it simple!
template < typename T, std::size_t Size, bool HasUnexpectedItemversion = false >    
class serializable_array
{
    public:
        BOOST_STATIC_ASSERT( Size > 0 );
        serializable_array() : m_count( Size ) {}
        typedef boost::array< T, Size > Type;
        const Type& operator()() const { return m_value; }
        Type& operator()() { return m_value; }
    
    private:
        std::size_t m_count;
        Type m_value;
    
        friend class boost::serialization::access;
        template < typename U, std::size_t S >
        friend std::ostream& operator<<( std::ostream &os, const serializable_array< U, S >& gp );

        template< class Archive >
        void serialize( Archive & ar, const unsigned int /* file_version */ )
        {
            ar & boost::serialization::make_nvp( "count", m_count );
            serializable_array_item< boost::is_class< T >::value || HasUnexpectedItemversion >::add_item_version( ar ); // quick and dirty, hate it (see comment to this class above)
            for( std::size_t i = 0; i < Size; ++i )
            {
                ar & boost::serialization::make_nvp( "item", m_value[i] );
            }
        }
};

struct position_type
{
    impl::serializable_array< float, 3 > xyz;
    
    template< class Archive >
    void serialize( Archive & ar, const unsigned int /* file_version */ ) { ar & BOOST_SERIALIZATION_NVP( xyz ); }    
};

struct orientation_type
{
    impl::serializable_array< float, 3 > rpy;
    
    template< class Archive >
    void serialize( Archive & ar, const unsigned int /* file_version */ ) { ar & BOOST_SERIALIZATION_NVP( rpy ); }    
};

struct color_type
{
    color_type() { rgb()[0] = rgb()[1] = rgb()[2] = 0; }
    impl::serializable_array< float, 3 > rgb;
    
    template< class Archive >
    void serialize( Archive & ar, const unsigned int /* file_version */ ) { ar & BOOST_SERIALIZATION_NVP( rgb ); }
};

struct px_type
{
    px_type()
        : id_( 0 )
        , rotCorrection_( 0 )
        , vertCorrection_( 0 )
        , distCorrection_( 0 )
        , vertOffsetCorrection_( 0 )
        , horizOffsetCorrection_( 0 )
    {
    }
    
    unsigned int id_;
    double rotCorrection_;
    double vertCorrection_;
    float distCorrection_;
    float vertOffsetCorrection_;
    float horizOffsetCorrection_;

    template< class Archive >
    void serialize( Archive & ar, const unsigned int /* file_version */ )
    {
        ar & BOOST_SERIALIZATION_NVP( id_ )
           & BOOST_SERIALIZATION_NVP( rotCorrection_ )
           & BOOST_SERIALIZATION_NVP( vertCorrection_ )
           & BOOST_SERIALIZATION_NVP( distCorrection_ )
           & BOOST_SERIALIZATION_NVP( vertOffsetCorrection_ )
           & BOOST_SERIALIZATION_NVP( horizOffsetCorrection_ );
    }
};

struct point_type
{
    px_type px;
    
    template< class Archive >
    void serialize( Archive & ar, const unsigned int /* file_version */ ) { ar & BOOST_SERIALIZATION_NVP( px ); }
};

struct serializable_db
{
    float distLSB_;
    impl::position_type position_;
    impl::orientation_type orientation_;
    impl::serializable_array< impl::color_type, 64 > colors_;
    impl::serializable_array< unsigned int, 64 > enabled_; // quick and dirty, bool does not work
    impl::serializable_array< unsigned char, 64 > intensity_;
    impl::serializable_array< unsigned char, 64, true > minIntensity_;
    impl::serializable_array< unsigned char, 64, true > maxIntensity_;
    impl::serializable_array< impl::point_type, 64 > points_;
    
    friend class boost::serialization::access;
    friend std::ostream & operator<<( std::ostream &os, const serializable_db &gp );

    template< class Archive >
    void serialize( Archive & ar, const unsigned int /* file_version */ )
    {
        ar & BOOST_SERIALIZATION_NVP( distLSB_ )
           & BOOST_SERIALIZATION_NVP( position_ )
           & BOOST_SERIALIZATION_NVP( orientation_ )
           & BOOST_SERIALIZATION_NVP( colors_ )
           & BOOST_SERIALIZATION_NVP( enabled_ )
           & BOOST_SERIALIZATION_NVP( intensity_ )
           & BOOST_SERIALIZATION_NVP( minIntensity_ )
           & BOOST_SERIALIZATION_NVP( maxIntensity_ )
           & BOOST_SERIALIZATION_NVP( points_ );
    }    
};

} } } // namespace snark {  namespace velodyne { namespace impl {

#endif // SNARK_SENSORS_VELODYNE_IMPL_SERIALIZABLEDB_H_
