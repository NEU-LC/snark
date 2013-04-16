// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2011 The University of Sydney
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. All advertising materials mentioning features or use of this software
//    must display the following acknowledgement:
//    This product includes software developed by the The University of Sydney.
// 4. Neither the name of the The University of Sydney nor the
//    names of its contributors may be used to endorse or promote products
//    derived from this software without specific prior written permission.
//
// NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
// GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
// HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS OR IMPLIED
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
// IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


#ifndef SNARK_PERCEPTION_PIN_SCREEN_HEADER_GUARD_
#define SNARK_PERCEPTION_PIN_SCREEN_HEADER_GUARD_

#include <cassert>
#include <map>
#include <Eigen/Core>

namespace snark {

/// A 2D grid which stores in each cell a stl vector
/// @todo this is a legacy code copy-pasted just to
///       make refactoring possible elsewhere
///       improve and refactor, once needed
template < typename T >
class pin_screen
{
    public:
        /// value type
        typedef T value_type;

        /// index type
        typedef Eigen::Matrix< std::size_t, 1, 3 > index_type;

        /// size type
        typedef Eigen::Matrix< std::size_t, 1, 2 > size_type;

        /// column type
        typedef std::map< std::size_t, T > column_type;

        /// constructor
        pin_screen( std::size_t size1 , std::size_t size2 );

        /// constructor
        pin_screen( size_type size );

        /// return 2D array size
        size_type size() const { return size_type( m_grid.rows(), m_grid.cols() ); }

        /// return column
        /// @todo add non-const column()?
        const column_type& column( std::size_t i , std::size_t j ) const { return m_grid( i, j ); }

        /// return column height
        std::size_t height( std::size_t i , std::size_t j ) const;

        /// return true, if element exists
        bool exists( std::size_t i , std::size_t j , std::size_t k ) const;

        /// return true, if element exists
        bool exists( const index_type& i ) const { return exists( i[0], i[1], i[2] ); }

        /// return reference to element, if exists
        T* find( std::size_t i , std::size_t j , std::size_t k );

        /// return reference to element, if exists
        T* find( const index_type& i ) { return find( i[0], i[1], i[2] ); }

        /// return reference to element, if exists
        const T* find( std::size_t i , std::size_t j , std::size_t k ) const;

        /// return reference to element, if exists
        const T* find( const index_type& i ) const { return find( i[0], i[1], i[2] ); }

        /// return reference to element; creates element, if it does not exist
        T& operator()( std::size_t i , std::size_t j , std::size_t k );

        /// return reference to element; crashes, if element does not exist (thus check Exists() first )
        const T& operator()( std::size_t i , std::size_t j , std::size_t k ) const;

        /// same as touch(); return reference to element; creates element, if it does not exist
        /// @todo it may be unsafe, since sometimes
        ///       compiler may decide to use non-const version
        ///       when you mean const
        T& operator()( const index_type& i ) { return operator()( i[0], i[1], i[2] ); }

        /// return reference to element; crashes, if element does not exist (thus check Exists() first )
        const T& operator()( const index_type& i ) const { return operator()( i[0], i[1], i[2] ); }

        /// return reference to element, create, if it does not exist
        T& touch( std::size_t i , std::size_t j , std::size_t k ) { return m_grid( i, j )[k]; }

        /// return reference to element, create, if it does not exist
        T& touch( const index_type& i ) { return touch( i[0], i[1], i[2] ); }

        /// erase element
        void erase( std::size_t i, std::size_t j, std::size_t k );

        /// erase element
        void erase( const index_type& i ) { return erase( i[0], i[1], i[2] ); }

        /// clean
        void clear();

        /// iterator, quick and dirty
        class iterator;

        /// const iterator, quick and dirty
        class const_iterator;

        /// return begin
        iterator begin();

        /// return begin
        const_iterator begin() const;

        /// return end
        iterator end();

        /// return end
        const_iterator end() const;

        /// neighbourhood iterator
        /// @todo real quick and dirty, just no time; definitely we need to go for a better abstration
        class neighbourhood_iterator;

    private:
        friend class iterator;
        friend class const_iterator;
        typedef ::Eigen::Matrix< column_type, ::Eigen::Dynamic, ::Eigen::Dynamic > GridType;
        GridType m_grid;
};

template < typename T >
class pin_screen< T >::const_iterator
{
    public:
        /// value type
        typedef T value_type;

        /// index type
        typedef typename pin_screen< T >::index_type index_type;

        /// size type
        typedef typename pin_screen< T >::size_type size_type;

        /// dimensions
        enum { Dimensions = 3 };

        /// operators (add more as needed)
        bool operator==( const const_iterator& rhs ) const { return m_grid == rhs.m_grid && m_column == rhs.m_column && m_it == rhs.m_it; }
        bool operator!=( const const_iterator& rhs ) const { return !operator==( rhs ); }
        bool operator<( const const_iterator& rhs ) const
        {
            assert( m_grid == rhs.m_grid );
            if (m_column[0] < rhs.m_column[0]) return true;
            if (m_column[0] > rhs.m_column[0]) return false;

            if (m_column[1] < rhs.m_column[1]) return true;
            if (m_column[1] > rhs.m_column[1]) return false;

            if (m_it->first < rhs.m_it->first) return true;
            if (m_it->first > rhs.m_it->first) return false;

            return false;

            // partially ordered operator, which does not seem to work for RB tree containers
            //return m_column[0] < rhs.m_column[0] || m_column[1] < rhs.m_column[1] || m_it->first < rhs.m_it->first;
        }
        const T& operator*() const { return m_it->second; }
        const T* operator->() const { return &m_it->second; }
        index_type operator()() const { return index_type( m_column[0], m_column[1], m_it->first ); }
        const const_iterator& operator++()
        {
            if( m_it != ( *m_grid )( m_column[0], m_column[1] ).end() ) { ++m_it; }
            while( m_it == ( *m_grid )( m_column[0], m_column[1] ).end() )
            {
                ++m_column[1];
                if( m_column[1] >= std::size_t( m_grid->cols() ) )
                {
                    ++m_column[0];
                    if( m_column[0] >= std::size_t( m_grid->rows() ) ) { return *this; }
                    m_column[1] = 0;
                }
                m_it = ( *m_grid )( m_column[0], m_column[1] ).begin();
            }
            return *this;
        }

        const_iterator() : m_column( 0, 0 ) {}

    protected:
        friend class pin_screen< T >;
        friend class pin_screen< T >::iterator;
        const GridType* m_grid;
        size_type m_column;
        typename pin_screen< T >::column_type::const_iterator m_it;
};

/// pin screen iterator
template < typename T >
class pin_screen< T >::iterator
{
    public:
        /// value type
        typedef T value_type;

        /// index type
        typedef typename pin_screen< T >::index_type index_type;

        /// size type
        typedef typename pin_screen< T >::size_type size_type;

        /// dimensions
        enum { Dimensions = 3 };

        /// operators (add more as needed)
        bool operator==( const iterator& rhs ) const { return m_grid == rhs.m_grid && m_column == rhs.m_column && m_it == rhs.m_it; }
        bool operator!=( const iterator& rhs ) const { return !operator==( rhs ); }
        bool operator<( const const_iterator& rhs ) const
        {
            assert( m_grid == rhs.m_grid );
            if (m_column[0] < rhs.m_column[0]) return true;
            if (m_column[0] > rhs.m_column[0]) return false;

            if (m_column[1] < rhs.m_column[1]) return true;
            if (m_column[1] > rhs.m_column[1]) return false;

            if (m_it->first < rhs.m_it->first) return true;
            if (m_it->first > rhs.m_it->first) return false;

            return false;

            // partially ordered operator, which does not seem to work for RB tree containers
            //return m_column[0] < rhs.m_column[0] || m_column[1] < rhs.m_column[1] || m_it->first < rhs.m_it->first;
        }
        T& operator*() { return m_it->second; }
        T* operator->() { return &m_it->second; }
        const T& operator*() const { return m_it->second; }
        const T* operator->() const { return &m_it->second; }
        index_type operator()() const { return index_type( m_column[0], m_column[1], m_it->first ); }
        const iterator& operator++()
        {
            if( m_it != ( *m_grid )( m_column[0], m_column[1] ).end() ) { ++m_it; }
            while( m_it == ( *m_grid )( m_column[0], m_column[1] ).end() )
            {
                ++m_column[1];
                if( m_column[1] >= std::size_t( m_grid->cols() ) )
                {
                    ++m_column[0];
                    if( m_column[0] >= std::size_t( m_grid->rows() ) ) { return *this; }
                    m_column[1] = 0;
                }
                m_it = ( *m_grid )( m_column[0], m_column[1] ).begin();
            }
            return *this;
        }

        operator const_iterator() const
        {
            const_iterator it;
            it.m_grid = m_grid;
            it.m_column = m_column;
            it.m_it = m_it;
            return it;
        }

        iterator() : m_column( 0, 0 ) {}

    protected:
        friend class pin_screen< T >;
        GridType* m_grid;
        size_type m_column;
        typename pin_screen< T >::column_type::iterator m_it;
};

template < typename T >
inline typename pin_screen< T >::iterator pin_screen< T >::begin()
{
    iterator it;
    it.m_grid = &m_grid;
    it.m_it = m_grid( 0, 0 ).begin();
    if( m_grid( 0, 0 ).empty() ) { ++it; }
    return it;
}

template < typename T >
inline typename pin_screen< T >::const_iterator pin_screen< T >::begin() const
{
    const_iterator it;
    it.m_grid = &m_grid;
    it.m_it = m_grid( 0, 0 ).begin();
    if( m_grid( 0, 0 ).empty() ) { ++it; }
    return it;
}

template < typename T >
inline typename pin_screen< T >::iterator pin_screen< T >::end()
{
    iterator it;
    it.m_grid = &m_grid;
    it.m_column = size_type( m_grid.rows(), m_grid.cols() );
    it.m_it = m_grid( m_grid.rows() - 1, m_grid.cols() - 1 ).end();
    return it;
}

template < typename T >
inline typename pin_screen< T >::const_iterator pin_screen< T >::end() const
{
    const_iterator it;
    it.m_grid = &m_grid;
    it.m_column = size_type( m_grid.rows(), m_grid.cols() );
    it.m_it = m_grid( m_grid.rows() - 1, m_grid.cols() - 1 ).end();
    return it;
}

/// matrix neighbourhood iterator
template < typename T >
class pin_screen< T >::neighbourhood_iterator : public pin_screen< T >::iterator
{
    public:
        /// itself
        typedef typename pin_screen< T >::neighbourhood_iterator iterator;
        
        /// index type
        typedef typename pin_screen< T >::iterator::index_type index_type;

        /// increment
        const neighbourhood_iterator& operator++();

        /// return neighbourhood center
        //const iterator& Centre() const { return m_center; }

        /// return begin
        static neighbourhood_iterator begin( const typename pin_screen< T >::iterator& center );

        /// return end
        static neighbourhood_iterator end( const typename pin_screen< T >::iterator& center );

    private:
        //iterator m_center;
        index_type m_center;
        index_type m_begin;
        index_type m_end;
        using pin_screen< T >::iterator::m_grid;
        using pin_screen< T >::iterator::m_column;
        using pin_screen< T >::iterator::m_it;
        void Init( const typename pin_screen< T >::iterator& center );
};

template < typename T >
inline void pin_screen< T >::neighbourhood_iterator::Init( const typename pin_screen< T >::iterator& center )
{
    m_grid = center.m_grid;
    m_center = center();
    m_begin[0] = m_center[0] - ( m_center[0] > 0 ? 1 : 0 );
    m_begin[1] = m_center[1] - ( m_center[1] > 0 ? 1 : 0 );
    m_begin[2] = m_center[2] - ( m_center[2] > 0 ? 1 : 0 );
    m_end[0] = m_center[0] + 1 + ( m_center[0] < std::size_t( m_grid->rows() ) - 1 ? 1 : 0 );
    m_end[1] = m_center[1] + 1 + ( m_center[1] < std::size_t( m_grid->cols() ) - 1 ? 1 : 0 );
    m_end[2] = m_center[2] + 1 + 1; // pin screen can grow upwards without limits
}

template < typename T >
inline const typename pin_screen< T >::neighbourhood_iterator& pin_screen< T >::neighbourhood_iterator::operator++()
{
    if( m_it != ( *m_grid )( m_column[0], m_column[1] ).end() && m_it->first < m_end[2] ) { ++m_it; }
    while( m_it == ( *m_grid )( m_column[0], m_column[1] ).end() || m_it->first >= m_end[2] || this->operator()() == m_center )
    {
        ++m_column[1];
        if( m_column[1] >= m_end[1] )
        {
            ++m_column[0];
            if( m_column[0] >= m_end[0] )
            {
                m_it = ( *( m_grid ) )( m_end[0] - 1, m_end[1] - 1 ).end();
                return *this;
            }
            m_column[1] = m_begin[1];
        }
        m_it =   m_begin[2] == 0
               ? ( *m_grid )( m_column[0], m_column[1] ).begin()
               : ( *m_grid )( m_column[0], m_column[1] ).upper_bound( m_begin[2] - 1 );
    }
    return *this;
}

template < typename T >
inline typename pin_screen< T >::neighbourhood_iterator pin_screen< T >::neighbourhood_iterator::begin( const typename pin_screen< T >::iterator& center )
{
    neighbourhood_iterator it;
    it.Init( center );
    for( it.m_column[0] = it.m_begin[0]; it.m_column[0] < it.m_end[0]; ++it.m_column[0] )
    {
        for( it.m_column[1] = it.m_begin[1]; it.m_column[1] < it.m_end[1]; ++it.m_column[1] )
        {
            if( it.m_begin[2] == 0 )
            {
                it.m_it = ( *( it.m_grid ) )( it.m_column[0], it.m_column[1] ).begin();
            }
            else
            {
                it.m_it = ( *( it.m_grid ) )( it.m_column[0], it.m_column[1] ).upper_bound( it.m_begin[2] - 1 );
            }
            for( ; it.m_it != ( *( it.m_grid ) )( it.m_column[0], it.m_column[1] ).end() && it.m_it->first < it.m_end[2]; ++it.m_it )
            {
                if( it() != it.m_center ) { return it; }
            }
        }
    }
    it.m_it = ( *( it.m_grid ) )( it.m_end[0] - 1, it.m_end[1] - 1 ).end();
    return it;
}

template < typename T >
inline typename pin_screen< T >::neighbourhood_iterator pin_screen< T >::neighbourhood_iterator::end( const typename pin_screen< T >::iterator& center )
{
    neighbourhood_iterator it;
    it.Init( center );
    it.m_column[0] = it.m_end[0];
    it.m_column[1] = it.m_end[1];
    it.m_it = ( *( it.m_grid ) )( it.m_end[0] - 1, it.m_end[1] - 1 ).end();
    return it;
}

template < typename T >
inline pin_screen< T >::pin_screen( std::size_t size1 , std::size_t size2 )
    : m_grid( size1, size2 )
{
}

template < typename T >
inline pin_screen< T >::pin_screen( typename pin_screen< T >::size_type size )
    : m_grid( size[0], size[1] )
{
}

template < typename T >
inline std::size_t pin_screen< T >::height( std::size_t i , std::size_t j ) const
{
    return m_grid( i, j ).empty() ? 0 : m_grid( i, j ).rbegin()->first;
}

template < typename T >
inline bool pin_screen< T >::exists( std::size_t i , std::size_t j , std::size_t k ) const
{
    return m_grid( i, j ).find( k ) != m_grid( i, j ).end();
}

template < typename T >
inline T* pin_screen< T >::find( std::size_t i , std::size_t j , std::size_t k )
{
    typename column_type::iterator it( m_grid( i, j ).find( k ) );
    return it == m_grid( i, j ).end() ? NULL : &it->second;
}

template < typename T >
inline const T* pin_screen< T >::find( std::size_t i, std::size_t j, std::size_t k ) const
{
    typename column_type::const_iterator it( m_grid( i, j ).find( k ) );
    return it == m_grid( i, j ).end() ? NULL : &it->second;
}

template < typename T >
inline T& pin_screen< T >::operator() ( std::size_t i, std::size_t j, std::size_t k )
{
    return touch( i, j, k );
}

template < typename T >
inline const T& pin_screen< T >::operator() ( std::size_t i, std::size_t j, std::size_t k ) const
{
    return *find( i, j, k );
}

template< typename T >
inline void pin_screen< T >::erase( std::size_t i, std::size_t j, std::size_t k )
{
    m_grid( i, j ).erase( k );
}

template< typename T >
inline void pin_screen< T >::clear()
{
    for( std::size_t i = 0; i < m_grid.rows(); ++i )
    {
        for( std::size_t j = 0; j < m_grid.cols(); ++j )
        {
            m_grid( i, j ).clear();
        }
    }
}

} // namespace snark {

#endif // #ifndef SNARK_PERCEPTION_PIN_SCREEN_HEADER_GUARD_
