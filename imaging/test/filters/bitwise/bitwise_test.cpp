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
#include <comma/base/exception.h>

#include <gtest/gtest.h>

#include <boost/algorithm/string/erase.hpp>
#include <boost/container/vector.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/algorithm/string/erase.hpp>
#include <boost/algorithm/string/replace.hpp>
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
        result_type op_and( const result_type & opl, const result_type & opr ) const { return [ opl, opr ]( std::ostream & os ) -> std::ostream & { os << '('; opl( os ); os << " & "; opr( os ); os << ')'; return os; }; }
        result_type op_or(  const result_type & opl, const result_type & opr ) const { return [ opl, opr ]( std::ostream & os ) -> std::ostream & { os << '('; opl( os ); os << " | "; opr( os ); os << ')'; return os; }; }
        result_type op_xor( const result_type & opl, const result_type & opr ) const { return [ opl, opr ]( std::ostream & os ) -> std::ostream & { os << '('; opl( os ); os << " ^ "; opr( os ); os << ')'; return os; }; }
        result_type op_not( const result_type & op ) const { return [ op ]( std::ostream & os ) -> std::ostream & { os << "(~"; op( os ); os << ')'; return os; }; }
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

    namespace global
    {
    const std::vector< std::string > inputs =
        {
            "a and b",
            "a or b",
            "a xor b",
            "not a",
            "not a and b",
            "not (a and b)",
            "a or b or c",
            "(a and b) xor ((c and d) or (a and b))",
            "a and b xor (c and d or a and b)",
            // "a and b xor (c and f(d+g) or a and b)",
            // "a and f(d+g)",
            // "(a and f(d+g)) or c",
            // "f:(d+g)/a|foo:bar",
        };

    const std::vector< std::string > expected =
        {
            "(a & b)",
            "(a | b)",
            "(a ^ b)",
            "(~a)",
            "((~a) & b)",
            "(~(a & b))",
            "(a | (b | c))",
            "((a & b) ^ ((c & d) | (a & b)))",
            "((a & b) ^ ((c & d) | (a & b)))",
            // "((a & b) ^ ((c & f(d+g)) | (a & b)))",
            // "(a & f(d+g))",
            // "((a & f(d+g)) | c)",
            // "f:(d+g)/a|foo:bar",
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
    } // namespace global

    int call_direct( const lookup_map_t< int > & m, const boost::function< int( int, int, int, int ) > & f ) { return f( m.at("a"), m.at("b"), m.at("c"), m.at("d") ); }

    std::vector< lookup_map_t< int > > lookup_ints = {
        { { "a",  135 }, { "b",   84 }, { "c",  213 }, { "d",  104 } },
        { { "a",   13 }, { "b", 2983 }, { "c", -676 }, { "d", 9238 } },
        { { "a", 4567 }, { "b", -837 }, { "c", 9652 }, { "d",  -38 } },
    };

} // anonymous

namespace snark{ namespace cv_mat {

namespace bitwise
{

    namespace brackets
    {

        struct bracket;

        typedef boost::variant< std::string
                              , boost::recursive_wrapper< bracket >
                              > element;

        typedef boost::container::vector< element > sequence;

        struct bracket
        {
           explicit bracket( const sequence & f = sequence() ) : s_( f ) {}
           sequence s_;
        };

        struct printer : boost::static_visitor< void >
        {
            printer( std::ostream & os ) : _os(os) {}
            std::ostream & _os;

            void operator()( const std::string & s ) const { _os << s; }
            void operator()( const bracket & b ) const { _os << '('; ( *this )( b.s_ ); _os << ')'; }
            void operator()( const element & e ) const { boost::apply_visitor( *this, e ); }
            void operator()( const sequence & s ) const { for ( const auto e : s ) { boost::apply_visitor( *this, e ); } }
        };

        inline std::ostream & operator<<( std::ostream & os, const element & l ) {
            boost::apply_visitor( printer( os ), l );
            return os;
        }

        inline std::ostream & operator<<( std::ostream & os, const sequence & s ) {
            for ( const auto & e : s ) { os << e; }
            return os;
        }

        struct resolver : boost::static_visitor< expr >
        {
            resolver( const std::map< std::string, expr > & m ) : m_( m ) {}
            const std::map< std::string, expr > & m_;

            expr operator()( const std::string & s ) const {
                for ( const auto & e : m_ )
                {
                    if ( s == e.first ) { return e.second; }
                    std::size_t pos = s.find( e.first );
                    if ( pos == std::string::npos ) { continue; }
                    if ( e.second.which() != 0 ) { COMMA_THROW( comma::exception, "in string '" << s << "' value " << e.first << " is a logical expression '" << e.second << "', not a string" ); }
                    const std::string & replacement = '(' + boost::get< std::string >( e.second ) + ')';
                    const std::string & news = boost::replace_all_copy( s, e.first, replacement );
                    return (*this)( news );
                }
                // never resolved anything
                return expr( s );
            }

            template< typename Op >
            expr operator()( const binary_op< Op > & b ) const {
                expr opl = boost::apply_visitor( *this, b.oper1 );
                expr opr = boost::apply_visitor( *this, b.oper2 );
                return binary_op< Op >( opl, opr );
            }
            expr operator()( const unary_op< op_not > & u ) const {
                expr op = boost::apply_visitor( *this, u.oper1 );
                return unary_op< op_not >( op );
            }
        };

        struct visitor : boost::static_visitor< void >
        {
            visitor() : next_( 0 ), name_( visitor::get_name() ), subexpressions_(), value_() {}

            void operator()( const std::string & s ) {
                if ( s == "and" ) { value_ += " and "; return; }
                if ( s == "xor" ) { value_ += " xor "; return; }
                if ( s == "or"  ) { value_ += " or " ; return; }
                if ( s == "not" ) { value_ += " not "; return; }
                value_ += s;
            }
            void operator()( const bracket & b ) {
                visitor inner;
                expr sub = inner.process( b.s_ );
                std::string id = name_ + "_" + boost::lexical_cast< std::string >( next_++ );
                subexpressions_[ id ] = sub;
                value_ += id;
            }
            void operator()( const element & e ) { boost::apply_visitor( *this, e ); }

            expr process( const sequence & s ) {
                for ( const auto e : s ) { boost::apply_visitor( *this, e ); }
                auto f( std::begin( value_ ) ), l( std::end( value_ ) );
                parser< decltype( f ) > p;
                expr result;
                bool ok = boost::spirit::qi::phrase_parse( f, l, p, boost::spirit::qi::space, result );
                if ( !ok ) { COMMA_THROW( comma::exception, "cannot parse the string '" << value_ << "'" ); }
                if ( f != l ) { COMMA_THROW( comma::exception, "string '" << value_ << "', unparsed remainder '" << std::string( f, l ) << "'" ); }
                resolver r( subexpressions_ );
                return boost::apply_visitor( r, result );
            }

            private:
                unsigned int next_;
                std::string name_;
                std::map< std::string, expr > subexpressions_;
                std::string value_;

                static std::string get_name()
                {
                    static boost::uuids::random_generator g_;
                    return "e" + boost::algorithm::erase_all_copy( boost::uuids::to_string( g_() ), "-" );
                }
        };

        template< typename It, typename Skipper = boost::spirit::qi::space_type >
        struct parser : boost::spirit::qi::grammar< It, sequence(), Skipper >
        {
            parser() : parser::base_type( sequence_ )
            {
                namespace qi = boost::spirit::qi;

                sequence_ = element_ >> *element_;
                element_ = ( bracket_ | var_ );
                bracket_ = '(' >> sequence_ >> ')';
                var_ = qi::lexeme[ +(qi::alnum | qi::char_("=;_,./:|+-")) ];

                BOOST_SPIRIT_DEBUG_NODE( sequence_ );
                BOOST_SPIRIT_DEBUG_NODE( element_ );
                BOOST_SPIRIT_DEBUG_NODE( bracket_ );
                BOOST_SPIRIT_DEBUG_NODE( var_ );
            }

            private:
                boost::spirit::qi::rule< It, std::string(), Skipper > var_;
                boost::spirit::qi::rule< It, bracket(), Skipper > bracket_;
                boost::spirit::qi::rule< It, element(), Skipper > element_;
                boost::spirit::qi::rule< It, sequence(), Skipper > sequence_;
        };

    } // namespace brackets

} // namespace bitwise

} } // namespace snark{ namespace cv_mat {

TEST( bitwise, brackets )
{
    const std::vector< std::string > inputs =
        {
            "f:(d+g)/a|foo:bar",
            "ratio:(r + b)/(1.0+g)|threshold:otsu, 1.2e-8|foo:bar",
            "foo=(ratio:(r+b)/(1.0+g)|threshold:otsu,1.2e-8|foo:bar)/(bar)",
            "foo=( ratio:(r+b)/(1.0 +g )|threshold:otsu,1.2e-8|foo:bar)/ (bar, baz)",
            "foo=( ratio:(r+b)/(1.0 +g )|threshold:otsu,1.2e-8|foo:bar)/ (bar, baz)and blah",
            "foo=( ratio:(r+b)/(1.0 +g )|threshold:otsu,1.2e-8|foo:bar or linear-combination:r|threshold:10)/ (bar, baz)and blah",
            "( ratio:(r+b)/(1.0 +g )|threshold:otsu,1.2e-8|foo:bar or linear-combination:r|threshold:10) xor (bar, baz) and blah",
        };

    for ( size_t i = 0; i < inputs.size(); ++i )
    {
        auto f( std::begin( inputs[i] ) ), l( std::end( inputs[i] ) );
        brackets::parser< decltype( f ) > p;

        try {
            brackets::sequence result;
            bool ok = boost::spirit::qi::phrase_parse( f, l, p, boost::spirit::qi::space, result );
            EXPECT_TRUE( ok );
            EXPECT_EQ( f, l );
            // std::cerr << "ok? " << ok << ", f == l? " << ( f == l ) << ", unparsed: '" << std::string( f, l ) << "'" << std::endl;
            {
                std::ostringstream os;
                os << result;
            }
            brackets::visitor v;
            expr e = v.process( result );
            std::ostringstream os;
            os << "parse: " << inputs[i] << '\n';
            os << "result: " << e;
            std::cerr << os.str() << std::endl;
        }
        catch ( const boost::spirit::qi::expectation_failure< std::string::const_iterator > & e )
        {
            std::cerr << "expectation failure:" << std::endl;
            print_info( e.what_ );
        }
        catch ( const std::exception & e )
        {
            std::cerr << "exception:" << std::endl;
            std::cerr << e.what() << std::endl;
        }
    }
}

TEST( bitwise, printer )
{
    for ( size_t i = 0; i < global::inputs.size(); ++i )
    {
        auto f( std::begin( global::inputs[i] ) ), l( std::end( global::inputs[i] ) );
        logical::parser< decltype( f ) > p;

        try {
            expr result;
            bool ok = boost::spirit::qi::phrase_parse( f, l, p, boost::spirit::qi::space, result );
            EXPECT_TRUE( ok );
            EXPECT_EQ( f, l );
            {
                std::ostringstream os;
                os << result;
                // std::cerr << expected[i] << " : " << os.str() << std::endl;
                EXPECT_EQ( os.str(), global::expected[i] );
            }
        } catch ( const boost::spirit::qi::expectation_failure< std::string::const_iterator > & e )
        {
            std::cerr << "exception:" << std::endl;
            print_info( e.what_ );
        }
    }
}

TEST( bitwise, special )
{
    const std::string & f0 = "linear-combination:r|convert-to:ub|threshold:otsu,1.5";
    const std::string & f1 = "ratio:(r - g)/(r + g)|convert-to:ub|threshold:1.2e-1";
    const std::vector< std::string > inputs =
        {
            f0 + " and " + f1,
        };

    const std::vector< std::string > expected =
        {
            "(" + boost::algorithm::erase_all_copy( f0, " " ) + " & " + boost::algorithm::erase_all_copy( f1, " " ) + ")",
        };

    for ( size_t i = 0; i < inputs.size(); ++i )
    {
        const auto & s = tabify_bitwise_ops( inputs[i] );
        auto f( std::begin( s ) ), l( std::end( s ) );
        logical::parser< decltype( f ) > p;

        expr result;
        bool ok = boost::spirit::qi::phrase_parse( f, l, p, boost::spirit::qi::space, result );
        EXPECT_TRUE( ok );
        EXPECT_EQ( f, l );
        {
            std::ostringstream os;
            os << result;
            // std::cerr << expected[i] << " : " << os.str() << std::endl;
            EXPECT_EQ( os.str(), expected[i] );
        }
    }
}

TEST( bitwise, writer )
{
    for ( size_t i = 0; i < global::inputs.size(); ++i )
    {
        auto f( std::begin( global::inputs[i] ) ), l( std::end( global::inputs[i] ) );
        logical::parser< decltype( f ) > p;

        expr result;
        bool ok = boost::spirit::qi::phrase_parse( f, l, p, boost::spirit::qi::space, result );
        EXPECT_TRUE( ok );
        EXPECT_EQ( f, l );
        {
            std::ostringstream os;
            writer w;
            auto scribe = boost::apply_visitor( visitor< std::ostream &, std::ostream &, writer >( w ), result );
            scribe( os );
            EXPECT_EQ( os.str(), global::expected[i] );
        }
    }
}

TEST( bitwise, logical_int )
{
    for ( size_t i = 0; i < global::inputs.size(); ++i )
    {
        auto f( std::begin( global::inputs[i] ) ), l( std::end( global::inputs[i] ) );
        logical::parser< decltype( f ) > p;

        expr result;
        bool ok = boost::spirit::qi::phrase_parse( f, l, p, boost::spirit::qi::space, result );
        EXPECT_TRUE( ok );
        EXPECT_EQ( f, l );
        {
            for ( const auto & m : lookup_ints )
            {
                logician< int > l( m );
                auto worker = boost::apply_visitor( visitor< boost::none_t, int, logician< int > >( l ), result );
                int r = worker( boost::none );
                int q = call_direct( m, global::direct[i] );
                EXPECT_EQ( r, q );
            }
        }
    }
}

TEST( bitwise, logical_matrix )
{
    std::vector< lookup_map_t< cv::Mat > > lookup_matrices;
    for ( const auto & m : lookup_ints )
    {
        lookup_map_t< cv::Mat > matrices;
        for ( const auto & k : { "a", "b", "c", "d" } )
        {
            cv::Mat matrix( 3, 4, CV_16UC1, cv::Scalar(0) );
            matrix.at< comma::int16 >( 2, 3 ) = m.at( k );
            matrices[ k ] = matrix.clone();
        }
        lookup_matrices.push_back( matrices );
    }
    for ( size_t i = 0; i < global::inputs.size(); ++i )
    {
        auto f( std::begin( global::inputs[i] ) ), l( std::end( global::inputs[i] ) );
        logical::parser< decltype( f ) > p;

        expr result;
        bool ok = boost::spirit::qi::phrase_parse( f, l, p, boost::spirit::qi::space, result );
        EXPECT_TRUE( ok );
        EXPECT_EQ( f, l );
        {
            for ( size_t j = 0; j < lookup_matrices.size(); ++j )
            {
                logician< cv::Mat > l( lookup_matrices[j] );
                auto worker = boost::apply_visitor( visitor< boost::none_t, cv::Mat, logician< cv::Mat > >( l ), result );
                cv::Mat r = worker( boost::none );
                int ri = r.at< comma::int16 >(2, 3);
                int q = call_direct( lookup_ints[j], global::direct[i] );
                EXPECT_EQ( ri, q );
            }
        }
    }
}

TEST( bitwise, tabify )
{
    for ( size_t i = 0; i < global::inputs.size(); ++i )
    {
        const std::string & s = tabify_bitwise_ops( global::inputs[i] );
        auto f( std::begin( s ) ), l( std::end( s ) );
        logical::parser< decltype( f ) > p;

        expr result;
        bool ok = boost::spirit::qi::phrase_parse( f, l, p, boost::spirit::qi::space, result );
        EXPECT_TRUE( ok );
        EXPECT_EQ( f, l );
        {
            std::ostringstream os;
            os << result;
            EXPECT_EQ( os.str(), global::expected[i] );
        }
    }
}
