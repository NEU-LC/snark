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
// 3. Neither the name of the University of Sydney nor the
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

#include "../../../../imaging/cv_mat/detail/bitwise.h"
#include <comma/base/types.h>

#include <gtest/gtest.h>

#include <boost/algorithm/string/erase.hpp>
#include <opencv2/core/core.hpp>

#include <iostream>

using namespace snark::cv_mat;
using namespace snark::cv_mat::bitwise;
using boost::spirit::ascii::space;

namespace {
    struct info_printer
    {
	typedef boost::spirit::utf8_string string;

	void element( string const & tag, string const & value, int depth ) const
	{
	    for ( int i = 0; i < ( depth*4 ); ++i ) { // indent to depth
		std::cerr << ' ';
            }

	    std::cerr << "tag: " << tag;
	    if ( value != "" ) { std::cerr << ", value: " << value; }
	    std::cerr << std::endl;
	}
    };

    void print_info(boost::spirit::info const& what)
    {
	using boost::spirit::basic_info_walker;

	info_printer pr;
	basic_info_walker< info_printer > walker( pr, what.tag, 0 );
	boost::apply_visitor( walker, what.value );
    }

    struct writer
    {
        typedef boost::static_visitor< boost::function< std::ostream & ( std::ostream & ) > >::result_type result_type;

        result_type term( const std::string & s ) const { return [ &s ]( std::ostream & os ) -> std::ostream & { os << s; return os; }; }
        result_type op_and( const result_type & opl, const result_type & opr ) const { return [ opl, opr ]( std::ostream & os ) -> std::ostream & { os << '['; opl( os ); os << " & "; opr( os ); os << ']'; return os; }; }
        result_type op_or(  const result_type & opl, const result_type & opr ) const { return [ opl, opr ]( std::ostream & os ) -> std::ostream & { os << '['; opl( os ); os << " | "; opr( os ); os << ']'; return os; }; }
        result_type op_xor( const result_type & opl, const result_type & opr ) const { return [ opl, opr ]( std::ostream & os ) -> std::ostream & { os << '['; opl( os ); os << " ^ "; opr( os ); os << ']'; return os; }; }
        result_type op_not( const result_type & op ) const { return [ op ]( std::ostream & os ) -> std::ostream & { os << "[~"; op( os ); os << ']'; return os; }; }
    };

    template< typename T >
    using lookup_map_t = std::map< std::string, T >;

    template< typename T >
    struct logician
    {
        logician( const lookup_map_t< T > & m ) : m_( m ) {}
        const lookup_map_t< T > & m_;

        typedef typename boost::static_visitor< boost::function< T ( boost::none_t ) > >::result_type result_type;

        result_type term( const std::string & s ) const { return [ &s, m = m_ ]( boost::none_t ) -> T { return m.at( s ); }; }
        result_type op_and( const result_type & opl, const result_type & opr ) const { return [ opl, opr ]( boost::none_t ) -> T { const T & il = opl( boost::none ); const T & ir = opr( boost::none ); return il & ir; }; }
        result_type op_or(  const result_type & opl, const result_type & opr ) const { return [ opl, opr ]( boost::none_t ) -> T { const T & il = opl( boost::none ); const T & ir = opr( boost::none ); return il | ir; }; }
        result_type op_xor( const result_type & opl, const result_type & opr ) const { return [ opl, opr ]( boost::none_t ) -> T { const T & il = opl( boost::none ); const T & ir = opr( boost::none ); return il ^ ir; }; }
        result_type op_not( const result_type & op ) const { return [ op ]( boost::none_t ) -> T { const T & i = op( boost::none ); return ~i; }; }
    };

    const std::vector< std::string > inputs =
        {
            // "a and b",
            // "a or b",
            // "a xor b",
            // "not a",
            // "not a and b",
            // "not (a and b)",
            // "a or b or c",
            // "(a and b) xor ((c and d) or (a and b))",
            // "a and b xor (c and d or a and b)",
            // "a and b xor (c and f(d+g) or a and b)",
            // "a and f(d+g)",
            // "(a and f(d+g)) or c",
            "f:(d + g) / a|foo:bar",
        };

    const std::vector< std::string > expected =
        {
            // "[a & b]",
            // "[a | b]",
            // "[a ^ b]",
            // "[~a]",
            // "[[~a] & b]",
            // "[~[a & b]]",
            // "[a | [b | c]]",
            // "[[a & b] ^ [[c & d] | [a & b]]]",
            // "[[a & b] ^ [[c & d] | [a & b]]]",
            // "[[a & b] ^ [[c & f(d+g)] | [a & b]]]",
            // "[a and f(d+g)]",
            // "[[a and f(d+g)] or c]",
            "f:(d + g) / a|foo:bar",
        };

    const std::vector< boost::function< int( int, int, int, int ) > > direct =
        {
            []( int a, int b, int c, int d ) ->int { return (a & b); },
            []( int a, int b, int c, int d ) ->int { return (a | b); },
            []( int a, int b, int c, int d ) ->int { return (a ^ b); },
            []( int a, int b, int c, int d ) ->int { return (~a); },
            []( int a, int b, int c, int d ) ->int { return ((~a) & b); },
            []( int a, int b, int c, int d ) ->int { return (~(a & b)); },
            []( int a, int b, int c, int d ) ->int { return (a | (b | c)); },
            []( int a, int b, int c, int d ) ->int { return ((a & b) ^ ((c & d) | (a & b))); },
            []( int a, int b, int c, int d ) ->int { return ((a & b) ^ ((c & d) | (a & b))); },
        };

    int call_direct( const lookup_map_t< int > & m, const boost::function< int( int, int, int, int ) > & f ) { return f( m.at("a"), m.at("b"), m.at("c"), m.at("d") ); }

    std::vector< lookup_map_t< int > > lookup_ints = {
        { { "a",  135 }, { "b",   84 }, { "c",  213 }, { "d",  104 } },
        { { "a",   13 }, { "b", 2983 }, { "c", -676 }, { "d", 9238 } },
        { { "a", 4567 }, { "b", -837 }, { "c", 9652 }, { "d",  -38 } },
    };

} // anonymous

namespace testing
{
    struct bracket
    {
       explicit bracket( const std::string & s = "" ) : s_( s ) { }
       std::string s_;
    };

    typedef boost::variant< std::string
                          , bracket
                          > leaf;

    struct filter : public std::vector< leaf > {};

    struct printer : boost::static_visitor< void >
    {
        printer( std::ostream & os ) : _os(os) {}
        std::ostream & _os;

        void operator()( const std::string & s ) const { _os << s; }
        void operator()( const bracket & b ) const { _os << '(' << b.s_ << ')'; }
        void operator()( const leaf & l ) const { boost::apply_visitor( *this, l ); }
    };

    inline std::ostream & operator<<( std::ostream & os, const leaf & l ) {
        boost::apply_visitor( printer( os ), l );
        return os;
    }

    inline std::ostream & operator<<( std::ostream & os, const filter & s ) {
        for ( const auto & e : s ) { os << e; }
        return os;
    }

    template< typename It, typename Skipper = boost::spirit::qi::space_type >
    struct parser : boost::spirit::qi::grammar< It, filter(), Skipper >
    {
        parser() : parser::base_type( filter_ )
        {
            namespace qi = boost::spirit::qi;

            filter_ = var_ >> *leaf_;
            leaf_ = ( bracket_ | var_ );
            bracket_ = '(' >> var_ >> ')';
            var_ = qi::lexeme[ +(qi::alnum | qi::char_(",./:|+-")) ];

            BOOST_SPIRIT_DEBUG_NODE( filter_ );
            BOOST_SPIRIT_DEBUG_NODE( leaf_ );
            BOOST_SPIRIT_DEBUG_NODE( bracket_ );
            BOOST_SPIRIT_DEBUG_NODE( var_ );
        }

        private:
            boost::spirit::qi::rule< It, std::string(), Skipper > var_;
            boost::spirit::qi::rule< It, bracket(), Skipper > bracket_;
            boost::spirit::qi::rule< It, leaf(), Skipper > leaf_;
            boost::spirit::qi::rule< It, filter(), Skipper > filter_;
    };

} // namespace testing

TEST( bitwise, testing )
{
    const std::vector< std::string > inputs =
        {
            "f:(d+g)/a|foo:bar",
            "ratio:(r+b)/(1.0+g)|threshold:otsu,1.2e-8|foo:bar",
        };

    const std::vector< std::string > expected =
        {
            "f:(d+g)/a|foo:bar",
            "ratio:(r+b)/(1.0+g)|threshold:otsu,1.2e-8|foo:bar",
        };

    for ( size_t i = 0; i < inputs.size(); ++i )
    {
        auto f( std::begin( inputs[i] ) ), l( std::end( inputs[i] ) );
        testing::parser< decltype( f ) > p;

        try {
            testing::filter result;
            bool ok = boost::spirit::qi::phrase_parse( f, l, p, boost::spirit::qi::space, result );
            EXPECT_TRUE( ok );
            EXPECT_EQ( f, l );
            {
                std::ostringstream os;
                os << result;
                // std::cerr << "expected '" << expected[i] << "'" << std::endl;
                // std::cerr << "got      '" << os.str()    << "'" << std::endl;
                EXPECT_EQ( os.str(), expected[i] );
            }
        }
        catch ( const boost::spirit::qi::expectation_failure< std::string::const_iterator > & e )
        {
            std::cerr << "exception:" << std::endl;
            print_info( e.what_ );
        }
    }
}

// TEST( bitwise, printer )
// {
//     for ( size_t i = 0; i < inputs.size(); ++i )
//     {
//         auto f( std::begin( inputs[i] ) ), l( std::end( inputs[i] ) );
//         parser< decltype( f ) > p;
// 
//         try {
//         expr result;
//         bool ok = boost::spirit::qi::phrase_parse( f, l, p, boost::spirit::qi::space, result );
//         // EXPECT_TRUE( ok );
//         // EXPECT_EQ( f, l );
//         {
//             std::ostringstream os;
//             os << result;
//             std::cerr << expected[i] << " : " << os.str() << std::endl;
//             // EXPECT_EQ( os.str(), expected[i] );
//         }
//         } catch ( const boost::spirit::qi::expectation_failure< std::string::const_iterator > & e )
//         {
//             std::cerr << "exception:" << std::endl;
//             print_info( e.what_ );
//         }
//     }
// }

// TEST( bitwise, special )
// {
//     const std::string & f0 = "linear-combination:r|convert-to:ub|threshold:otsu,1.5";
//     const std::string & f1 = "ratio:(r - g)/(r + g)|convert-to:ub|threshold:1.2e-1";
//     const std::vector< std::string > inputs =
//         {
//             f0 + " and " + f1,
//         };
// 
//     const std::vector< std::string > expected =
//         {
//             "(" + boost::algorithm::erase_all_copy( f0, " " ) + " & " + boost::algorithm::erase_all_copy( f1, " " ) + ")",
//         };
// 
//     for ( size_t i = 0; i < inputs.size(); ++i )
//     {
//         const auto & s = tabify_bitwise_ops( inputs[i] );
//         auto f( std::begin( s ) ), l( std::end( s ) );
//         parser< decltype( f ) > p;
// 
//         expr result;
//         bool ok = boost::spirit::qi::phrase_parse( f, l, p, boost::spirit::qi::space, result );
//         // EXPECT_TRUE( ok );
//         // EXPECT_EQ( f, l );
//         std::cerr << "ok? " << ok << ", f == l? " << ( f == l ) << std::endl;
//         {
//             std::ostringstream os;
//             os << result;
//             std::cerr << expected[i] << " : " << os.str() << std::endl;
//             // EXPECT_EQ( os.str(), expected[i] );
//         }
//     }
// }

// TEST( bitwise, writer )
// {
//     for ( size_t i = 0; i < inputs.size(); ++i )
//     {
//         auto f( std::begin( inputs[i] ) ), l( std::end( inputs[i] ) );
//         parser< decltype( f ) > p;
// 
//         expr result;
//         bool ok = boost::spirit::qi::phrase_parse( f, l, p, boost::spirit::qi::space, result );
//         EXPECT_TRUE( ok );
//         EXPECT_EQ( f, l );
//         {
//             std::ostringstream os;
//             writer w;
//             auto scribe = boost::apply_visitor( visitor< std::ostream &, std::ostream &, writer >( w ), result );
//             scribe( os );
//             EXPECT_EQ( os.str(), expected[i] );
//         }
//     }
// }

// TEST( bitwise, logical_int )
// {
//     for ( size_t i = 0; i < inputs.size(); ++i )
//     {
//         auto f( std::begin( inputs[i] ) ), l( std::end( inputs[i] ) );
//         parser< decltype( f ) > p;
// 
//         expr result;
//         bool ok = boost::spirit::qi::phrase_parse( f, l, p, boost::spirit::qi::space, result );
//         EXPECT_TRUE( ok );
//         EXPECT_EQ( f, l );
//         {
//             for ( const auto & m : lookup_ints )
//             {
//                 logician< int > l( m );
//                 auto worker = boost::apply_visitor( visitor< boost::none_t, int, logician< int > >( l ), result );
//                 int r = worker( boost::none );
//                 int q = call_direct( m, direct[i] );
//                 EXPECT_EQ( r, q );
//             }
//         }
//     }
// }

// TEST( bitwise, logical_matrix )
// {
//     std::vector< lookup_map_t< cv::Mat > > lookup_matrices;
//     for ( const auto & m : lookup_ints )
//     {
//         lookup_map_t< cv::Mat > matrices;
//         for ( const auto & k : { "a", "b", "c", "d" } )
//         {
//             cv::Mat matrix( 3, 4, CV_16UC1, cv::Scalar(0) );
//             matrix.at< comma::int16 >( 2, 3 ) = m.at( k );
//             matrices[ k ] = matrix.clone();
//         }
//         lookup_matrices.push_back( matrices );
//     }
//     for ( size_t i = 0; i < inputs.size(); ++i )
//     {
//         auto f( std::begin( inputs[i] ) ), l( std::end( inputs[i] ) );
//         parser< decltype( f ) > p;
// 
//         expr result;
//         bool ok = boost::spirit::qi::phrase_parse( f, l, p, boost::spirit::qi::space, result );
//         EXPECT_TRUE( ok );
//         EXPECT_EQ( f, l );
//         {
//             for ( size_t j = 0; j < lookup_matrices.size(); ++j )
//             {
//                 logician< cv::Mat > l( lookup_matrices[j] );
//                 auto worker = boost::apply_visitor( visitor< boost::none_t, cv::Mat, logician< cv::Mat > >( l ), result );
//                 cv::Mat r = worker( boost::none );
//                 int ri = r.at< comma::int16 >(2, 3);
//                 int q = call_direct( lookup_ints[j], direct[i] );
//                 EXPECT_EQ( ri, q );
//             }
//         }
//     }
// }

// TEST( bitwise, tabify )
// {
//     for ( size_t i = 0; i < inputs.size(); ++i )
//     {
//         const std::string & s = tabify_bitwise_ops( inputs[i] );
//         auto f( std::begin( s ) ), l( std::end( s ) );
//         parser< decltype( f ) > p;
// 
//         expr result;
//         bool ok = boost::spirit::qi::phrase_parse( f, l, p, boost::spirit::qi::space, result );
//         EXPECT_TRUE( ok );
//         EXPECT_EQ( f, l );
//         {
//             std::ostringstream os;
//             os << result;
//             EXPECT_EQ( os.str(), expected[i] );
//         }
//     }
// }
