// Copyright (c) 2021 Vsevolod Vlaskine

#pragma once

#include <string>
#include <QColor>
#include <comma/application/command_line_options.h>

namespace snark { namespace graphics { namespace plotting {

struct series // quick and dirty
{
    struct config
    {
        std::string chart;
        std::string color_name;
        QColor color;
        bool scroll; // todo! a better name!
        std::string shape;
        std::string style;
        std::string title;
        float weight;
        
        config(): scroll( false ), weight( 1 ) {}
        
        config( const comma::command_line_options& options );
    };
};
    
} } } // namespace snark { namespace graphics { namespace plotting {
