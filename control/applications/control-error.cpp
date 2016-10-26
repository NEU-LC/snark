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

#include <iostream>
#include <boost/optional.hpp>
#include <boost/bimap.hpp>
#include <boost/assign.hpp>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/csv/options.h>
#include <comma/csv/stream.h>
#include <comma/csv/names.h>
#include <comma/csv/traits.h>
#include <comma/io/select.h>
#include <comma/io/stream.h>
#include <comma/name_value/name_value.h>
#include <comma/math/cyclic.h>
#include "../wayline.h"
#include "control.h"
#include "traits.h"

static const std::string name = "control-error";

template< typename T > std::string field_names( bool full_xpath = false ) { return comma::join( comma::csv::names< T >( full_xpath ), ',' ); }
template< typename T > std::string format( bool full_xpath = false, const std::string& fields = "" ) { return comma::csv::format::value< T >( !fields.empty() ? fields : field_names< T >( full_xpath ), full_xpath ); }

static const double default_proximity = 0.1;
static const std::string default_mode = "fixed";

typedef snark::control::control_error_output_t output_t;

static void usage( bool verbose = false )
{
    std::cerr << std::endl;
    std::cerr << "take target waypoints on stdin and feedback from address and output to stdout with the appended wayline heading and control errors" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: " << name << " <feedback> [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "feedback: \"<address>[;<options>]\"" << std::endl;
    std::cerr << "    address: address where the feedback stream is published, e.g. tcp:localhost:12345" << std::endl;
    std::cerr << "    options: semicolon-separated csv options of the feedback stream, e.g. fields=t,x,y,,,,yaw;binary=t,6d" << std::endl;
    std::cerr << "    default fields: " << field_names< snark::control::feedback_t >() << std::endl;
    std::cerr << std::endl;
    if( verbose )
    {
        std::cerr << "csv stream options: " << std::endl;
        std::cerr << comma::csv::options::usage( field_names< snark::control::target_t >() ) << std::endl;
    }
    std::cerr << std::endl;
    std::cerr << "options: " << std::endl;
    std::cerr << "    --mode,-m=<mode>: control mode (default: " << default_mode << ")" << std::endl;
    std::cerr << "    --proximity,-p=<proximity>: a wayline is traversed as soon as current position is within proximity of the endpoint (default: " << default_proximity << ")" << std::endl;
    std::cerr << "    --past-endpoint: a wayline is traversed as soon as current position is past the endpoint (or proximity condition is met)" << std::endl;
    std::cerr << "    --frequency,-f=<frequency>: control frequency (the rate at which " << name <<" outputs control errors using latest feedback)" << std::endl;
    std::cerr << "    --heading-is-absolute: interpret target heading as global by default" << std::endl;
    std::cerr << "    --input-fields: show default input stream fields and exit" << std::endl;
    std::cerr << "    --format,--input-format: show binary format of default input stream fields and exit" << std::endl;
    std::cerr << "    --output-format: show binary format of output stream and exit (for wayline and control error fields only)" << std::endl;
    std::cerr << "    --output-fields: show output fields and exit (for wayline and control error fields only)" << std::endl;
    std::cerr << "    --strict: fail if a record with the timestamp earlier than the previous one is encountered (default: skip offending records)" << std::endl;
    std::cerr << std::endl;
    std::cerr << "control modes: " << std::endl;
    std::cerr << "    fixed: wait until the current waypoint is reached before accepting a new waypoint (first feedback position is the start of the first wayline)" << std::endl;
    std::cerr << "    dynamic: use a new waypoint as soon as it becomes available to define a new wayline form the current position to the new waypoint" << std::endl;
    std::cerr << std::endl;
    std::cerr << "default input fields: position/x,position/y,heading,is_absolute" << std::endl;
    std::cerr << "    required fields: position/x,position/y" << std::endl;
    std::cerr << "    optional fields: heading,is_absolute (default values: 0,0)" << std::endl;
    std::cerr << std::endl;
    std::cerr << "examples: " << std::endl;
    std::cerr << "    cat targets.bin | " << name << " \"tcp:localhost:12345;fields=t,x,y,,,,yaw;binary=t,6d\" --fields=x,y,,,speed --binary=3d,ui,d --past-endpoint" << std::endl;
    std::cerr << std::endl;
    exit( 1 );
}

std::string serialise( const snark::control::wayline::position_t& p )
{
    std::stringstream s;
    s << std::fixed << std::setprecision(12) << p.x() << ',' << p.y();
    return s.str();
}

enum control_mode_t { fixed, dynamic };

typedef boost::bimap< control_mode_t, std::string > named_mode_t;
static const named_mode_t named_modes = boost::assign::list_of< named_mode_t::relation >
    ( fixed, "fixed" )
    ( dynamic, "dynamic" );

control_mode_t mode_from_string( std::string s )
{
    if( !named_modes.right.count( s ) ) { COMMA_THROW( comma::exception, "control mode '" << s << "' is not found" ); };
    return  named_modes.right.at( s );
}

std::string mode_to_string( control_mode_t m ) { return  named_modes.left.at( m ); }

class wayline_follower
{
public:
    wayline_follower( control_mode_t mode, double proximity, bool use_past_endpoint, bool strict_time, double eps=1e-6 )
        : mode_( mode )
        , proximity_( proximity )
        , use_past_endpoint_( use_past_endpoint )
        , reached_( false )
        , no_previous_targets_( true )
        , strict_time_( strict_time )
        , eps_( eps )
        {
            if( proximity_ < 0 ) { COMMA_THROW( comma::exception, "expected positive proximity, got " << proximity_ ); }
        }

    void set_target( const snark::control::target_t& target, const snark::control::wayline::position_t& current_position )
    {
        snark::control::wayline::position_t from = ( mode_ == fixed && target_ ) ? target_->position : current_position;
        target_ = target;
        no_previous_targets_ = false;
        reached_ = ( current_position - target_->position ).norm() < proximity_;
        if( reached_ || ( from - target_->position ).norm() < eps_ ) { return; } // if from is too close to the new target, the old wayline will be used
        wayline_ = snark::control::wayline( from, target_->position );
    }
    void update( const std::pair< snark::control::feedback_t, std::string >& feedback )
    {
        if( reached_ ) { return; }
        if( previous_update_time_ && feedback.first.t < previous_update_time_ )
        {
            if( strict_time_ ) { COMMA_THROW( comma::exception, "received feedback time " << feedback.first.t << " that is earlier than the previous feedback time " << previous_update_time_ ); }
            return;
        }
        previous_update_time_ = feedback.first.t;
        feedback_buffer_ = feedback.second;
        reached_ = ( ( feedback.first.position - target_->position ).norm() < proximity_ )
            || ( use_past_endpoint_ && wayline_.is_past_endpoint( feedback.first.position ) );
        if( reached_ ) { return; }
        error_.cross_track = wayline_.cross_track_error( feedback.first.position );
        error_.heading = target_->is_absolute ? comma::math::cyclic< double >( comma::math::interval< double >( -M_PI, M_PI ), target_->heading - feedback.first.yaw )()
            : wayline_.heading_error( feedback.first.yaw, target_->heading );
    }
    bool target_reached() const { return reached_; }
    bool no_target() const { return no_previous_targets_ || reached_; }
    snark::control::error_t error() const { return error_; }
    snark::control::wayline::position_t to() const { return target_->position; }
    snark::control::wayline wayline() const { return wayline_; }
    std::string feedback() const { return feedback_buffer_; }

private:
    control_mode_t mode_;
    double proximity_;
    bool use_past_endpoint_;
    boost::optional< snark::control::target_t > target_;
    bool verbose_;
    bool reached_;
    bool no_previous_targets_;
    bool strict_time_;
    double eps_;
    snark::control::wayline wayline_;
    snark::control::error_t error_;
    boost::optional< boost::posix_time::ptime > previous_update_time_;
    std::string feedback_buffer_;
};

class full_output_t
{
public:
    full_output_t( const comma::csv::input_stream< snark::control::target_t >& input_stream,
              const comma::csv::input_stream< snark::control::feedback_t >& feedback_stream,
              comma::csv::output_stream< output_t >& output_stream,
              boost::optional< double > frequency )
        : output_stream_( output_stream )
    {
        if( ! ( input_stream.is_binary() && feedback_stream.is_binary() && output_stream_.is_binary() )
            && ! ( !input_stream.is_binary() && !feedback_stream.is_binary() && !output_stream_.is_binary() ) )
        {
            COMMA_THROW( comma::exception, "input, feedback, and output streams are not all binary or all ascii: "
                << "input is " << ( input_stream.is_binary() ? "binary" : "ascii" )
                << ", feedback is " << ( feedback_stream.is_binary() ? "binary" : "ascii" )
                << ", output is " << ( output_stream_.is_binary() ? "binary" : "ascii" ) )
        }
        is_binary_ = input_stream.is_binary();
        if( is_binary_ )
        {
            input_size_ = input_stream.binary().binary().format().size();
            feedback_size_ = feedback_stream.binary().binary().format().size();
        }
        else
        {
            if( input_stream.ascii().ascii().delimiter() != output_stream.ascii().ascii().delimiter()
                || input_stream.ascii().ascii().delimiter() != feedback_stream.ascii().ascii().delimiter() )
            {
                COMMA_THROW( comma::exception, "input, feedback, and output stream delimiters do not match" );
            }
            delimiter_ = output_stream.ascii().ascii().delimiter();
        }
        if( frequency )
        {
            if( *frequency <= 0 ) { COMMA_THROW( comma::exception, "expected positive frequency, got " << *frequency ); }
            delay_ = boost::posix_time::microseconds( static_cast< long >( 1000000 / *frequency ) );
            next_output_time_= boost::posix_time::microsec_clock::universal_time();
        }
    }
    void write( const std::string& input_buffer, const wayline_follower& follower )
    {
        if( ! ( follower.target_reached() 
                || ( next_output_time_ && boost::posix_time::microsec_clock::universal_time() > *next_output_time_ ) ) ) { return; }
        if( is_binary_ )
        {
            std::cout.write( &input_buffer[0], input_size_ );
            std::cout.write( &follower.feedback()[0], feedback_size_ );
        }
        else
        {
            std::cout << input_buffer << delimiter_;
            std::cout << comma::join( follower.feedback(), delimiter_ ) << delimiter_;
        }
        output_stream_.write( output_t( follower.wayline().heading(), follower.error(), follower.target_reached() ) );
        if( next_output_time_ ) { next_output_time_ = boost::posix_time::microsec_clock::universal_time() + delay_; }
     }
private:
    comma::csv::output_stream< output_t >& output_stream_;
    bool is_binary_;
    std::size_t input_size_;
    std::size_t feedback_size_;
    char delimiter_;
    boost::posix_time::time_duration delay_;
    boost::optional< boost::posix_time::ptime > next_output_time_;
};

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        comma::csv::options input_csv( options );
        comma::csv::input_stream< snark::control::target_t > input_stream( std::cin, input_csv, snark::control::target_t( options.exists( "--heading-is-absolute" ) ) );
        comma::csv::output_stream< output_t > output_stream( std::cout, input_csv.binary(), true, input_csv.flush, output_t() );
        if( options.exists( "--input-fields" ) ) { std::cout << field_names< snark::control::target_t >( true ) << std::endl; return 0; }
        if( options.exists( "--format,--input-format" ) ) { std::cout << format< snark::control::target_t >() << std::endl; return 0; }
        if( options.exists( "--output-fields" ) ) { std::cout << field_names< output_t >( true ) << std::endl; return 0; }
        if( options.exists( "--output-format" ) ) { std::cout << format< output_t >( true ) << std::endl; return 0; }
        control_mode_t mode = mode_from_string( options.value< std::string >( "--mode", default_mode ) );
        double proximity = options.value< double >( "--proximity", default_proximity );
        bool use_past_endpoint = options.exists( "--past-endpoint" );
        bool verbose = options.exists( "--verbose,-v" );
        bool strict = options.exists( "--strict" );
        std::vector< std::string > unnamed = options.unnamed( "--help,-h,--verbose,-v,--input-fields,--format,--input-format,--output-format,--output-fields,--past-endpoint,--heading-is-absolute,--strict", "-.*,--.*" );
        if( unnamed.empty() ) { std::cerr << name << ": feedback stream is not given" << std::endl; return 1; }
        comma::csv::options feedback_csv = comma::name_value::parser( "filename", ';', '=', false ).get< comma::csv::options >( unnamed[0] );
        comma::io::istream feedback_in( feedback_csv.filename, feedback_csv.binary() ? comma::io::mode::binary : comma::io::mode::ascii );
        comma::csv::input_stream< snark::control::feedback_t > feedback_stream( *feedback_in, feedback_csv );
        full_output_t output( input_stream, feedback_stream, output_stream, options.optional< double >( "--frequency,-f" ) );
        comma::io::select select;
        select.read().add( feedback_in );
        std::pair< snark::control::feedback_t, std::string > feedback;
        if( select.wait( boost::posix_time::seconds( 1 ) ) ) // TODO: consider using feedback timeout (when implemented) instead of the hardcoded 1 sec
        {
            const snark::control::feedback_t* p = feedback_stream.read();
            if( !p ) { std::cerr << name << ": feedback stream error" << std::endl; return 1; }
            feedback.first = *p;
            if( feedback_csv.binary() )
            {
                feedback.second.resize( feedback_csv.format().size() );
                ::memcpy( &feedback.second[0], feedback_stream.binary().last(), feedback.second.size());
            }
            else
            {
                feedback.second = comma::join( feedback_stream.ascii().last(), feedback_csv.delimiter );
            }
        }
        else
        {
            std::cerr << name << ": feedback is not publishing data" << std::endl;
            return 1;
        }
        //boost::posix_time::time_duration feedback_timeout; // todo
        select.read().add( comma::io::stdin_fd );
        std::deque< std::pair< snark::control::target_t, std::string > > targets;
        comma::signal_flag is_shutdown;
        wayline_follower follower( mode, proximity, use_past_endpoint, strict );
        while( !is_shutdown && std::cin.good() && std::cout.good() )
        {
            // todo? don't do select.check() on stdin in the loop or do it only in "dynamic" mode?
            while( !is_shutdown && ( input_stream.ready() || ( select.check() && select.read().ready( comma::io::stdin_fd ) ) ) )
            {
                const snark::control::target_t* p = input_stream.read();
                if( !p ) { break; }
                std::pair< snark::control::target_t, std::string > pair( *p, "" );
                if( input_csv.binary() )
                {
                    pair.second.resize( input_csv.format().size() );
                    ::memcpy( &pair.second[0], input_stream.binary().last(), pair.second.size() );
                }
                else
                {
                    pair.second = comma::join( input_stream.ascii().last(), input_csv.delimiter );
                }
                targets.push_back( pair );
            }
            while( !is_shutdown && ( feedback_stream.ready() || ( select.check() && select.read().ready( feedback_in ) ) ) )
            {
                const snark::control::feedback_t* p = feedback_stream.read();
                if( !p ) { std::cerr << name << ": end of feedback stream" << std::endl; return 1; }
                feedback.first = *p;
                if( feedback_csv.binary() )
                {
                    feedback.second.resize( feedback_csv.format().size() );
                    ::memcpy( &feedback.second[0], feedback_stream.binary().last(), feedback.second.size());
                }
                else
                {
                    feedback.second = comma::join( feedback_stream.ascii().last(), feedback_csv.delimiter );
                }
            }
            if( is_shutdown ) { break; }
            // todo: implement feedback timeout
            //if( !feedback->t.is_not_a_date_time() && ( feedback->t + feedback_timeout < now ) )
            //{ 
            //   if( verbose ) { warning }
            //   feedback.reset();
            //}
            if( targets.empty() ) { select.wait( boost::posix_time::millisec( 10 ) ); continue; }
            if( follower.no_target() || ( mode == dynamic && targets.size() > 1 ) )
            {
                if( mode == dynamic )
                {
                    std::pair< snark::control::target_t, std::string > pair = targets.back();
                    targets.clear();
                    targets.push_back( pair );
                }
                follower.set_target( targets.front().first, feedback.first.position );
                if( verbose ) { std::cerr << name << ": target waypoint " << serialise( follower.to() ) << ", current position " << serialise( feedback.first.position ) << std::endl; }
            }
            follower.update( feedback );
            output.write( targets.front().second, follower );
            if( follower.target_reached() )
            {
                if( verbose ) { std::cerr << name << ": reached waypoint " << serialise( follower.to() ) << ", current position " << serialise( feedback.first.position ) << std::endl; }
                targets.pop_front();
            }
        }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << name << ": " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << name << ": unknown exception" << std::endl; }
    return 1;
}
