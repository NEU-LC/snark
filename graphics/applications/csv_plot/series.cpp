// Copyright (c) 2021 Vsevolod Vlaskine

#include "series.h"

namespace snark { namespace graphics { namespace plotting { namespace series {

static const char* hex_color_( const std::string& c )
{
    if( c == "red" ) { return "#FF0000"; }
    if( c == "green" ) { return "#00FF00"; }
    if( c == "blue" ) { return "#0000FF"; }
    if( c == "yellow" ) { return "#FFFF00"; }
    if( c == "cyan" ) { return "#00FFFF"; }
    if( c == "magenta" ) { return "#FF00FF"; }
    if( c == "black" ) { return "#000000"; }
    if( c == "white" ) { return "#FFFFFF"; }
    if( c == "grey" ) { return "#888888"; }
    return &c[0];
}
    
config::config( const comma::command_line_options& options )
    : color_name( options.value< std::string >( "--color,--colour", "black" ) )
    , color( color_name.empty() ? QColor( 0, 0, 0 ) : QColor( hex_color_( color_name ) ) )
    , scroll( options.exists( "--scroll" ) )
    , shape( options.value< std::string >( "--shape,--type", "line" ) )
    , style( options.value< std::string >( "--style", "" ) )
    , weight( options.value( "--weight", 0.0 ) )
{
}

} } } } // namespace snark { namespace graphics { namespace plotting { namespace series {
